package frc.robot.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.constants.RobotConstants
import frc.robot.constants.ShootingConstants
import frc.robot.subsystems.hood.HoodConstants
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.utils.interpolation.InterpolatingDouble
import frc.robot.utils.interpolation.InterpolatingTreeMap
import frc.template.utils.seconds
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier
import kotlin.math.asin

data class ShootingParameters(
    val driveAngle                          : Angle,
    val hoodAngle                           : Angle,
    val shooterRPS                          : AngularVelocity,
    val futureEstimatedDistanceToTarget     : Distance,
    val currentDistanceToTarget             : Distance,
    val timeOfFlight                        : Time
)

/**
 * This class calculates everything regarding shooting, regardless of being static or moving.
 * Every subsystem parameter for shooting will be packed in a [ShootingParameters] object.
 * NOTE: Every property is a supplier to avoid creating unnecessary variables during calculations or requiring
 * access to [Superstructure].
 * @property fieldTargetTranslation The (x,y) coordinate of the target for shooting (HUB or bump in assist)
 * @property odometryPose The current pose of the robot
 * @property robotRelativeVelocity The current robot-relative [ChassisSpeeds]
 * @property robotFieldVelocity The current field-relative [ChassisSpeeds]
 * @property currentHoodAngle The current [frc.robot.subsystems.hood.Hood] angle
 */
class ShootingCalcs(
    val fieldTargetTranslation: Supplier<Translation2d>,
    val odometryPose: Supplier<Pose2d>,
    val robotRelativeVelocity: Supplier<ChassisSpeeds>,
    val robotFieldVelocity: Supplier<ChassisSpeeds>,
    val currentHoodAngle: Supplier<Angle>,
    val isAssist: Supplier<Boolean>
) {
    private val phaseDelay: Time = 0.03.seconds // From 6328's code. Assuming is 20ms cycle plus some tolerance.

    /*--------------------------------------- INTERPOLATION ---------------------------------------*/
    private val hoodScoreInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()
    private val shooterScoreInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()

    private val hoodAssistInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()
    private val shooterAssistInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()

    private val timeOfFlightInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()

    /*------------------------------------------- FILTERS -------------------------------------------*/
    // Filters were taken from 6328' Launch Calculator. Their purpose is to "remove" signal noise, in this case
    // to have smoother changes between Hood and Drive requested angles.
    // The parameter of LinearFilter.movingAverage() is the amount of samples for the averaged signal. In this case, 5.
    private val hoodFilter: LinearFilter = LinearFilter.movingAverage(
        (0.1 / RobotConstants.LoopInfo.loopPeriod.`in`(Seconds)).toInt()
    )

    private val driveAngleFilter: LinearFilter = LinearFilter.movingAverage(
        (0.1 / RobotConstants.LoopInfo.loopPeriod.`in`(Seconds)).toInt()
    )

    init {
        configureInterpolationPoints()
    }

    /**
     * Calculates all parameters for shooting, regardless of being static or moving.
     * For further details, read:
     * - https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
     * - https://team-190.github.io/190-Robot-Code-Standards/SOTM/
     * @return The required [ShootingParameters] to reach the target.
     */
    fun getLatestShootingParameters(): ShootingParameters {
        val estimatedPose = odometryPose.get().exp( // Estimated robot pose after phase delay
            Twist2d(
                robotRelativeVelocity.get().vxMetersPerSecond * phaseDelay.`in`(Seconds),
                robotRelativeVelocity.get().vyMetersPerSecond * phaseDelay.`in`(Seconds),
                robotRelativeVelocity.get().omegaRadiansPerSecond * phaseDelay.`in`(Seconds)
            )
        )

        // Converts robot coordinates to shooter coordinates
        val estimatedShooterPosition = estimatedPose.transformBy(RobotConstants.RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME)
        // Gets the distance between shooter coordinates and target coordinates (all field relative)
        val estimatedShooterToTargetDistance = fieldTargetTranslation.get().getDistance(estimatedShooterPosition.translation)

        // Assume robot's velocity during autonomous, else transform velocity accounting for rotation and offset
        val shooterVelocity = if (DriverStation.isAutonomous()) robotFieldVelocity.get() else transformVelocity(
            robotFieldVelocity.get(),
            RobotConstants.RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.translation,
            odometryPose.get().rotation
        )

        // Initialize future estimates. These will be updated in the for loop below
        var estimatedFutureTimeOfFlight = timeOfFlightInterpolation.get( // TODO() = Check if an alternate interpolation for passing is needed
            InterpolatingDouble(estimatedShooterToTargetDistance))!!.value
        var estimatedFutureShooterPose = estimatedShooterPosition
        var estimatedFutureShooterToTargetDistance = estimatedShooterToTargetDistance

        // One run for every millisecond in robot loop
        for (i in 1..20) {
            estimatedFutureTimeOfFlight = timeOfFlightInterpolation.get(
                InterpolatingDouble(estimatedFutureShooterToTargetDistance))!!.value

            val offsetX = shooterVelocity.vxMetersPerSecond * estimatedFutureTimeOfFlight
            val offsetY = shooterVelocity.vyMetersPerSecond * estimatedFutureTimeOfFlight

            estimatedFutureShooterPose = Pose2d(
                estimatedShooterPosition.translation.plus(Translation2d(offsetX, offsetY)),
                estimatedShooterPosition.rotation
            )
            estimatedFutureShooterToTargetDistance = fieldTargetTranslation.get().getDistance(estimatedFutureShooterPose.translation)
        }

        // Uses robot-to-shooter transform inverse to get back to robot pose
        val estimatedFutureRobotPose = estimatedFutureShooterPose.transformBy(
            Transform2d(
                RobotConstants.RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.translation,
                RobotConstants.RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.rotation)
                .inverse()
        )
        // The requested drive angle to shoot
        val requestedDriveAngle = getDriveAngleWithLauncherOffset(estimatedFutureRobotPose, fieldTargetTranslation.get())

        // Decides which interpolation to use based on isAssist supplier
        val requestedHoodAngle = if (isAssist.get()) hoodAssistInterpolation.get(
            InterpolatingDouble(estimatedFutureShooterToTargetDistance))!!.value
        else hoodScoreInterpolation.get(
            InterpolatingDouble(estimatedFutureShooterToTargetDistance))!!.value

        val requestedShooterRPS = if (isAssist.get()) shooterAssistInterpolation.get(
            InterpolatingDouble(estimatedFutureShooterToTargetDistance))!!.value
        else shooterScoreInterpolation.get(
            InterpolatingDouble(estimatedFutureShooterToTargetDistance))!!.value

        // Telemetry
        Logger.recordOutput(ShootingConstants.Telemetry.SHOOTING_REQUESTED_HOOD_ANGLE, requestedHoodAngle)
        Logger.recordOutput(ShootingConstants.Telemetry.SHOOTING_REQUESTED_DRIVE_ANGLE, requestedDriveAngle)
        Logger.recordOutput(ShootingConstants.Telemetry.SHOOTING_REQUESTED_SHOOTER_RPS, requestedShooterRPS)

        return ShootingParameters(
            driveAngle                      =   Degrees.of(requestedDriveAngle.degrees), // Converts Rotation2d to Angle
            hoodAngle                       =   Degrees.of(requestedHoodAngle),
            shooterRPS                      =   RotationsPerSecond.of(requestedShooterRPS),
            futureEstimatedDistanceToTarget =   Meters.of(estimatedFutureShooterToTargetDistance),
            currentDistanceToTarget         =   Meters.of(estimatedShooterToTargetDistance),
            timeOfFlight                    =   Seconds.of(estimatedFutureTimeOfFlight)
        )
    }

    /**
     * Intended to be used during [net.tecdroid.util.stateMachine.States.EmergencyShootState] to ensure we are able
     * to score UNDER THE TRENCH. Keep in mind the driver would have to align manually ( ~ 90 degrees),
     * while fully against the wall. This is NOT intended to shoot while moving.
     * @return The [ShootingParameters] for emergency shooting under the trench.
     */
    fun getUnderTheTrenchShootingParameters(): ShootingParameters {
        return ShootingParameters(
            driveAngle                      =   Degrees.of(odometryPose.get().rotation.degrees), // Converts Rotation2d to Angle
            hoodAngle                       =   ShootingConstants.DefaultParameters.underTheTrenchHoodAngle,
            shooterRPS                      =   ShootingConstants.DefaultParameters.underTheTrenchShooterRPS,
            futureEstimatedDistanceToTarget =   ShootingConstants.DefaultParameters.underTheTrenchDistanceToHUB,
            currentDistanceToTarget         =   ShootingConstants.DefaultParameters.underTheTrenchDistanceToHUB,
            timeOfFlight                    =   timeOfFlightInterpolation.get(
                InterpolatingDouble(ShootingConstants.DefaultParameters.underTheTrenchDistanceToHUB.`in`(Meters))
            )?.value?.seconds ?: 1.0.seconds
        )
    }


    private fun getDriveAngleWithLauncherOffset(
        robotPose: Pose2d, target: Translation2d
    ): Rotation2d {
        val fieldToHubAngle = target.minus(robotPose.translation).angle
        val hubAngle =
            Rotation2d(
                asin(
                    MathUtil.clamp(
                        RobotConstants.RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.translation.y
                                / target.getDistance(robotPose.translation),
                        -1.0,
                        1.0
                    )
                )
            )
        val driveAngle =
            fieldToHubAngle.plus(hubAngle).plus(RobotConstants.RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.rotation)
        return driveAngle
    }

    /**
     * Transforms a velocity along a translation.
     * Taken from 6328' GeomUtil.java
     *
     * @param velocity The original velocity
     * @param transform The transform to the new position
     * @param currentRotation The current rotation of the robot
     * @return The new velocity
     */
    private fun transformVelocity(
        velocity: ChassisSpeeds, transform: Translation2d, currentRotation: Rotation2d
    ): ChassisSpeeds {
        return ChassisSpeeds(
            velocity.vxMetersPerSecond
                    - velocity.omegaRadiansPerSecond
                    * (transform.x * currentRotation.sin
                    + transform.y * currentRotation.cos),
            velocity.vyMetersPerSecond
                    + velocity.omegaRadiansPerSecond
                    * (transform.x * currentRotation.cos
                    - transform.y * currentRotation.sin),
            velocity.omegaRadiansPerSecond
        )
    }

    /**
     * Configures each interpolation tree.
     * Must be called during [init].
     */
    private fun configureInterpolationPoints() {
        // Shooter
        for (point in ShooterConstants.Control.shooterScoreHighCurvatureInterpolationPoints) {
            shooterScoreInterpolation.put(
                InterpolatingDouble(point.key.`in`(Meters)),
                InterpolatingDouble(point.value.`in`(RotationsPerSecond))
            )
        }

        for (point in ShooterConstants.Control.shooterAssistInterpolationPoints) {
            shooterAssistInterpolation.put(
                InterpolatingDouble(point.key.`in`(Meters)),
                InterpolatingDouble(point.value.`in`(RotationsPerSecond))
            )
        }

        // Hood
        for (point in HoodConstants.Control.hoodScoreHighCurvatureInterpolationPoints) {
            hoodScoreInterpolation.put(
                InterpolatingDouble(point.key.`in`(Meters)),
                InterpolatingDouble(point.value.`in`(Degrees))
            )
        }

        for (point in HoodConstants.Control.hoodAssistInterpolationPoints) {
            hoodAssistInterpolation.put(
                InterpolatingDouble(point.key.`in`(Meters)),
                InterpolatingDouble(point.value.`in`(Degrees))
            )
        }

        // Time of flight
        for (point in ShootingConstants.Interpolation.timeOfFlightPoints) {
            timeOfFlightInterpolation.put(
                InterpolatingDouble(point.key.`in`(Meters)),
                InterpolatingDouble(point.value.`in`(Seconds))
            )
        }
    }

}