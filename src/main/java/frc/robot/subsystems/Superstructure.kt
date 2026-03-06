package frc.robot.subsystems

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.Waypoint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.DriveCommands
import frc.robot.constants.Constants.isFlipped
import frc.robot.constants.FieldConstants
import frc.robot.constants.FieldZones
import frc.robot.constants.RobotConstants
import frc.robot.constants.SwerveTunerConstants
import frc.robot.subsystems.drivetrain.Drive
import frc.robot.subsystems.drivetrain.GyroIOPigeon2
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.hood.HoodConstants
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.IntakeConstants
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOLimelight
import frc.template.utils.degrees
import frc.template.utils.meters
import frc.template.utils.radians
import frc.template.utils.rotationsPerSecond
import frc.template.utils.seconds
import org.littletonrobotics.junction.AutoLogOutput
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.pow

class Superstructure(private val controller: CommandXboxController) : Subsystem {



    /**
     * Drivetrain initialization. Creates a new [Drive] derived from the Advantage Kit's template, it already
     * computes all the necessary operations and code by just giving it the four modules configured via
     * Phoenix Tuner which gives us the [SwerveTunerConstants] file with all the constants, offsets and
     * necessary inversions. It also requires a [GyroIOPigeon2], a constants file is generated along with the
     * chassis ones.
     */
    private val drive   : Drive     = Drive(
        GyroIOPigeon2(),
        ModuleIOTalonFX(SwerveTunerConstants.FrontLeft),
        ModuleIOTalonFX(SwerveTunerConstants.FrontRight),
        ModuleIOTalonFX(SwerveTunerConstants.BackLeft),
        ModuleIOTalonFX(SwerveTunerConstants.BackRight)
    )

    /**
     * Vision declaration. Creates a new [Vision] instance derived from the Advantage kit's template.
     * Odometry calculations are done within the class. Various [VisionIOLimelight] and other cameras can be configured
     * via this subsystem. It also requires an [drive.addVisionMeasurement] method.
     */
    private val vision = Vision(
        drive::addVisionMeasurement,
        VisionIOLimelight(VisionConstants.frontLimelight,   { drive.rotation }),
        VisionIOLimelight(VisionConstants.leftLimelight,    { drive.rotation }),
        VisionIOLimelight(VisionConstants.rightLimelight,   { drive.rotation })
    )

    val intake  : Intake    = Intake()
    private val indexer : Indexer   = Indexer()
    private val shooter : Shooter   = Shooter()
    val hood    : Hood      = Hood()


    init {
        resetDrivePose().ignoringDisable(true)
        drive.defaultCommand = driveFollowingDriverInput()
    }

    fun setVisionThrottle(throttle: Double) {
        vision.setThrottle(throttle)
    }

    /**
     * TODO(): Update comment
     * Intended for TeleOp shooting. Starts by enabling the [Shooter] interpolation, waiting until the Swerve
     * is in place to shoot, then changing [Hood] interpolation from distance driven to shooter velocity driven,
     * and finally enabling the indexer to start the actual shooting.
     * @return A [SequentialCommandGroup] with the above specifications.
     */
    fun shootStateSequenceDefaultCMD(): Command {
        return ParallelCommandGroup(
            shootingStateShooterInterpolationCMD(), // Enables shooter interpolation
            shootingStateHoodInterpolationCMD(), // Enables hood interpolation
            WaitUntilCommand { shooter.getShooterAngularVelocityError() // Waits until required velocity is reached
                .lte(RobotConstants.Control.SHOOTER_VELOCITY_TOLERANCE) }.withTimeout(4.5.seconds)
                .andThen(shootingStateIndexerEnableCMD()) // Enables the indexer, both hopper and tower rollers
                .andThen(WaitCommand(1.0.seconds)
                    .andThen(shootingStateIntakeDanceCMD())) // Makes the deployable component go up and down to push any stocked FUEL
        )
    }

    fun shootStateSequenceWithoutOdometryCMD(): Command {
        return ParallelCommandGroup(
            noStateShootOnly(), // Enables shooter manual control
            noStateHoodOnly(), // Enables hood manual control
            WaitUntilCommand { shooter.getShooterAngularVelocityError() // Waits until required velocity is reached
                .lte(RobotConstants.Control.SHOOTER_VELOCITY_TOLERANCE) }.withTimeout(4.5.seconds)
                .andThen(noStateIndexerOnly()) // Enables the indexer, both hopper and tower rollers
                .andThen(WaitCommand(1.0.seconds) // Safety for deployable assuming full hopper
                    .andThen(shootingStateIntakeDanceCMD()))) // Makes the deployable component go up and down to push any stocked FUEL
    }

    /**
     * Intended to be Initial Command of Shooting State. Locks the Swerve angle so that it targets the HUB.
     * During this command, the controller's right joystick is not used. Translation is still under the driver's
     * control.
     * @return A [RunCommand] that locks the Swerve angle to target the HUB.
     */
    fun driveTargetingHUB(): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { -controller.leftY * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_Y_MULTIPLIER },
            { -controller.leftX * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_X_MULTIPLIER },
            ::getSwerveToHubAngle
        )
    }

    /**
     * Intended to be End Command of Shooting State. Gives back the full Swerve control to the driver.
     * @return A [RunCommand] with the Swerve's default command.
     */
    fun driveFollowingDriverInput(): Command {
        return DriveCommands.joystickDriveAtAngle (
            drive,
            { -controller.leftY * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_Y_MULTIPLIER },
            { -controller.leftX * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_X_MULTIPLIER },
            ::getAngleFromJoystick
        )
    }

    @AutoLogOutput(key = "Odometry/Joystick Target Angle")
    fun getAngleFromJoystick(): Angle {
        val atan2Rad = atan2(-controller.rightY, controller.rightX)
        return if (atan2Rad == 0.0) atan2Rad.plus(Math.PI).radians else atan2Rad.plus(Math.PI / 2).radians
    }

    fun resetDrivePose(): Command {
        return InstantCommand({ drive.pose = Pose2d(drive.pose.translation, Rotation2d.k180deg) }, drive)
    }

    fun followTrajectory(path: PathPlannerPath): Command {
        return drive.followTrajectory(path)
    }

    /**
     * Gets the HUB -> Swerve vector through [getHubToSwerveVectorComponents] and calculates its magnitude,
     * which equals the Swerve -> HUB distance.
     * - Think of it as a right triangle, then (x,y) are the sides and the hypotenuse is the vector's magnitude.
     * @return The Swerve -> HUB [Distance]
     */
    //@AutoLogOutput(key = "Odometry/Swerve Distance to HUB", unit = "meters")
    private fun getSwerveToHubDistance(): Distance {
        val HUB_TO_SWERVE_VECTOR = getHubToSwerveVectorComponents()
        val xComponentMeters = HUB_TO_SWERVE_VECTOR.first.`in`(Meters)
        val yComponentMeters = HUB_TO_SWERVE_VECTOR.second.`in`(Meters)
        return hypot(xComponentMeters.pow(2), yComponentMeters.pow(2)).meters
    }

    @AutoLogOutput(key = "Odometry/Swerve Distance to HUB", unit = "meters")
    private fun getSwerveToHubDistanceLog(): Double {
        val HUB_TO_SWERVE_VECTOR = getHubToSwerveVectorComponents()
        val xComponentMeters = HUB_TO_SWERVE_VECTOR.first.`in`(Meters)
        val yComponentMeters = HUB_TO_SWERVE_VECTOR.second.`in`(Meters)
        return hypot(xComponentMeters.pow(2), yComponentMeters.pow(2))
    }

    /**
     * Gets the HUB -> Swerve vector through [getHubToSwerveVectorComponents] and adds one third of the current
     * chassisSpeeds to account for the 20ms cycle delay. The new (x,y) components are then passed to [atan2] to
     * get the angle necessary for the Swerve to target the center point of the HUB.
     * @return The [Angle] at which the Swerve should be in order to target the center of the HUB.
     */
    private fun getSwerveToHubAngle(): Angle {
        val HUB_TO_SWERVE_VECTOR = getHubToSwerveVectorComponents()

        val xAdjustedWithVelocity = HUB_TO_SWERVE_VECTOR.first.plus(drive.chassisSpeeds.vxMetersPerSecond.div(2).meters)
        val yAdjustedWithVelocity = HUB_TO_SWERVE_VECTOR.second.plus(drive.chassisSpeeds.vyMetersPerSecond.div(2).meters)

        return atan2(yAdjustedWithVelocity.`in`(Meters), xAdjustedWithVelocity.`in`(Meters)).radians
    }

    /**
     * Intended to run during Shooting State. Enables the [Shooter] interpolation with respect to the HUB.
     * @return A [RunCommand] interpolating the [Shooter] velocity based on [getSwerveToHubDistance].
     */
    fun shootingStateShooterInterpolationCMD(): Command {
        return shooter.setInterpolatedVelocityCMD { getSwerveToHubDistance() }
    }

    /**
     * Intended to run during Shooting State. Switches the [Hood] interpolation from distance driven to velocity
     * driven, allowing it to correct for [Shooter] RPMs losses.
     * @return A [RunCommand] interpolating the [Hood] angle based on the [Shooter] velocity.
     */
    fun shootingStateHoodInterpolationCMD(): Command {
        return hood.setVelocityDrivenInterpolatedAngleCMD { shooter.getShooterAngularVelocity() }
    }

    fun shootingStateIndexerEnableCMD(): Command {
        return indexer.enableIndexerCMD()
    }


    /**
     * Due to hopper's design, there is the possibility of FUELS getting stuck during Shooting State. To correct this,
     * the deployable component of the intake must go backwards and upwards in intervals of 15 - 30 degrees, pushing
     * FUELS towards the indexer and therefore, the [Shooter].
     * @return A [RunCommand] that makes the intake "dance" by moving it 15 degrees up and down, with its maximum
     *      being the intake's retracted angle.
     */
    fun shootingStateIntakeDanceCMD(): Command {
        return SequentialCommandGroup(
            intake.setDeployableAngleOnly(IntakeConstants.RetractileAngles.DeployedAngle
                .plus(RobotConstants.Control.INTAKE_DEPLOYABLE_DANCE_DELTA)),
            WaitUntilCommand { intake.getDeployableError() < RobotConstants.Control.INTAKE_DEPLOYABLE_DANCE_TOLERANCE }
                .withTimeout(1.0.seconds),
            intake.setDeployableAngleOnly(IntakeConstants.RetractileAngles.DeployedAngle),
            WaitUntilCommand { intake.getDeployableError() < RobotConstants.Control.INTAKE_DEPLOYABLE_DANCE_TOLERANCE }
                .withTimeout(1.0.seconds))
            .repeatedly()
    }

    fun noStateIntakeDeployableOnlyEnable(): Command {
        return intake.setDeployableAngleOnly(IntakeConstants.RetractileAngles.DeployedAngle)
    }

    fun noStateIntakeDeployableOnlyDisable(): Command {
        return intake.setDeployableAngleOnly(IntakeConstants.RetractileAngles.RetractedAngle)
    }

    fun noStateIndexerOnly(): Command {
        return indexer.enableIndexerCMD()
    }

    fun noStateShootOnly(): Command {
        return shooter.setVelocityCMD { ShooterConstants.Tunables.enabledRPMs.get().div(60.0).rotationsPerSecond }
    }

    fun noStateHoodOnly(): Command {
        return hood.setAngleTunableCMD { HoodConstants.Tunables.motorAngle.get().degrees }
    }

    /**
     * Intended to run before Shooting State. To prepare the [Hood], distance driven interpolation is performed.
     * This interpolation method must change during Shooting State to correct for any RPM losses.
     * @return A [RunCommand] interpolating the [Hood] angle based on the distance from Swerve to HUB.
     */
    fun preShootingStateHoodInterpolationCMD(): Command {
        return hood.setDistanceDrivenInterpolatedAngleCMD { getSwerveToHubDistance() }
    }

    fun intakeStateCMD(): Command {
        return intake.enableIntakeCMD()
    }

    /**
     * Receives a list of waypoints and the target end rotation of the Swerve at the end of the created path, then
     * applies the path constraints stored in [RobotConstants.AutonomousConfigs] and sets the goal end velocity to 0.
     * @param waypoints The [Waypoint] list for the generated path to follow.
     * @param goalEndRotation The Swerve rotation at the [GoalEndState].
     * @return A [PathPlannerPath] that follows the list of waypoints with the predefined set of constraints and
     *      the target end rotation.
     */
    fun getOnTheFlyPathFromWaypoints(waypoints: List<Waypoint>, goalEndRotation: Angle): PathPlannerPath {
        val path = PathPlannerPath(
            waypoints,  // Given waypoints, must be constructed in a separate method.
            RobotConstants.AutonomousConfigs.onTheFlyPathConstraints,   // Standard constraints for every path.
            null,   // Only relevant for pre-planned paths, must be null since we don't know the starting state.
            GoalEndState(0.0, Rotation2d(goalEndRotation))  // Robot stops at the end. Given rotation is applied.
        )

        return path
    }

    /**
     * Determines the (single) waypoint necessary to go under the trench, based on the current position of the Swerve.
     * The logic goes as follows:
     * - If the current y coordinate of the Swerve is greater than half the Field y length, only the upper trenches are
     *      eligible (understand upper trenches as the left trenches from the blue alliance drivers perspective).
     *      Otherwise, only the lower trenches are eligible.
     * - If the current x coordinate of the Swerve is closer to the Neutral Zone, the waypoint should be towards
     *      the Alliance Zone. This also works the other way around.
     * - The rotation of the waypoint is 0, since it does not represent the Swerve rotation. That is set at the
     *      [GoalEndState].
     * @return The suitable [Waypoint] to go under the trench, after considering alliance and current position.
     */
    fun getTrenchOnTheFlyWaypointList(): List<Waypoint> {
        val yCoordinate = if (drive.pose.measureY > FieldConstants.Dimensions.FIELD_WIDTH_Y)
            FieldConstants.Trench.UPPER_TRENCH_Y else FieldConstants.Trench.LOWER_TRENCH_Y

        val xCoordinate = if (isNearNeutralZone())
            FieldConstants.Trench.ALLIANCE_ZONE_END_DELTA_X.plus(
                if (isFlipped.invoke()) FieldConstants.Trench.RED_TRENCH_CENTER_X else FieldConstants.Trench.BLUE_TRENCH_CENTER_X)
            else FieldConstants.Trench.NEUTRAL_ZONE_END_DELTA_X.plus(
                if (isFlipped.invoke()) FieldConstants.Trench.RED_TRENCH_CENTER_X else FieldConstants.Trench.BLUE_TRENCH_CENTER_X)

        return PathPlannerPath.waypointsFromPoses(
            Pose2d(xCoordinate, yCoordinate, Rotation2d.kZero),
        )
    }

    fun storeHood(): Command {
        return hood.setAngleCMD(HoodConstants.Control.HoodDownPosition)
    }

    fun disableIntake(): Command {
        return intake.disableIntakeCMD()
    }

    fun disableIndexer(): Command {
        return indexer.stopIndexerCMD()
    }

    fun disableShooter(): Command {
        return shooter.stopShooterCMD()
    }

    fun disableSubsystems(): Command {
        return ParallelCommandGroup(
            disableIndexer(),
            disableIntake(),
            disableShooter()
        )
    }

    fun coastSubsystems(): Command {
        return ParallelCommandGroup(
            intake.coastCMD(),
            hood.coastCMD()
        )
    }

    fun brakeSubsystems(): Command {
        return ParallelCommandGroup(
            intake.brakeCMD(),
            hood.brakeCMD()
        )
    }

    fun isInsideZone(zone: FieldZones): Boolean {
        return getRobotCurrentZone() == zone
    }

    /**
     * After getting the HUB's center point based on the current alliance, a vector is constructed by subtracting
     * the chassis (x,y) components from the HUB's (x,y) components.
     * - [Pair] is used instead of [java.util.Vector] since the first one is far more memory efficient.
     * @return a [Pair] containing the (x,y) components - in that order - of the HUB -> Swerve vector.
     */
    private fun getHubToSwerveVectorComponents(): Pair<Distance, Distance> {
        val centerHUB_X = if (isFlipped.invoke()) FieldConstants.HUB.RED_HUB_CENTER_X else FieldConstants.HUB.BLUE_HUB_CENTER_X
        val centerHUB_Y = if (isFlipped.invoke()) FieldConstants.HUB.RED_HUB_CENTER_Y else FieldConstants.HUB.BLUE_HUB_CENTER_Y

        return Pair(centerHUB_X.minus(drive.pose.measureX), centerHUB_Y.minus(drive.pose.measureY))
    }

    @AutoLogOutput(key = "Odometry/Current Zone")
    private fun getRobotCurrentZone(): FieldZones {
        val robotX = drive.pose.measureX

        return when {
            // If the robot's X coordinate is less than the blue trench x coordinate, then the robot is
            // inside the Blue Alliance Zone.
            robotX < FieldConstants.Trench.BLUE_TRENCH_CENTER_X -> FieldZones.BLUE_ALLIANCE_ZONE
            // If robot x coordinate is less than the red trench x coordinate, then the robot is in the Neutral
            // Zone as the following condition is indirectly evaluated:
            // Blue trench center X < robotX < red trench center X
            robotX < FieldConstants.Trench.RED_TRENCH_CENTER_X -> FieldZones.NEUTRAL_ZONE
            // If not in Blue Alliance or Neutral Zone, then robot's inside the Red Alliance Zone.
            else -> FieldZones.RED_ALLIANCE_ZONE
        }
    }

    /**
     * Determines whether the robot is closer to the Neutral Zone than the Alliance Zone. This is needed to obtain
     * the waypoints that go under the trench in teleop, which have to decide whether "under the trench" means
     * towards Neutral or Alliance Zone.
     * To determine this, the x coordinate of the chassis is compared to corresponding middle trench coordinate
     * as follows:
     * - In blue alliance, the chassis has to be past the trench middle point to be considered to be in Neutral Zone
     * - In red alliance, the chassis has to be behind the trench middle point to be considered to be in Neutral Zone
     * @return Whether the chassis is closest to the Neutral Zone.
     */
    private fun isNearNeutralZone(): Boolean {
        val middlePointTrench = if (isFlipped.invoke())
            FieldConstants.Trench.RED_TRENCH_CENTER_X else FieldConstants.Trench.BLUE_TRENCH_CENTER_X
        val xCoordinateChassis = drive.pose.measureX

        return if (isFlipped.invoke()) xCoordinateChassis < middlePointTrench
                    else xCoordinateChassis > middlePointTrench
    }
}