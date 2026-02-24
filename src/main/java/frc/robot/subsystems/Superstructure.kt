package frc.robot.subsystems

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.Waypoint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.DriveCommands
import frc.robot.constants.Constants.isFlipped
import frc.robot.constants.FieldConstants
import frc.robot.constants.RobotConstants
import frc.robot.subsystems.drivetrain.Drive
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.IntakeConstants
import frc.robot.subsystems.shooter.Shooter
import frc.template.utils.meters
import frc.template.utils.radians
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

class Superstructure(private val controller: XboxController, private val drive: Drive, private val intake: Intake,
                     private val indexer: Indexer, private val shooter: Shooter, private val hood: Hood) {

    /**
     * Intended for TeleOp shooting. Starts by enabling the [Shooter] interpolation, waiting until the Swerve
     * is in place to shoot, then changing [Hood] interpolation from distance driven to shooter velocity driven,
     * and finally enabling the indexer to start the actual shooting.
     * @return A [SequentialCommandGroup] with the above specifications.
     */
    fun shootSequenceCMD(): Command {
        return SequentialCommandGroup(
            shooter.setInterpolatedVelocityCMD(::getSwerveToHubDistance),
            WaitUntilCommand {
                abs(drive.pose.rotation.minus(Rotation2d(getSwerveToHubAngle())).degrees) <
                        RobotConstants.Control.DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING.`in`(Degrees) },
            hood.setVelocityDrivenInterpolatedAngleCMD(shooter::getShooterAngularVelocity),
            indexer.enableIndexerCMD()
        )
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
        return DriveCommands.joystickDrive(
            drive,
            { -controller.leftY * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_Y_MULTIPLIER },
            { -controller.leftX * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_X_MULTIPLIER },
            { controller.rightX * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_Z_MULTIPLIER }
        )
    }

    /**
     * Gets the HUB -> Swerve vector through [getHubToSwerveVectorComponents] and calculates its magnitude,
     * which equals the Swerve -> HUB distance.
     * - Think of it as a right triangle, then (x,y) are the sides and the hypotenuse is the vector's magnitude.
     * @return The Swerve -> HUB [Distance]
     */
    fun getSwerveToHubDistance(): Distance {
        val HUB_TO_SWERVE_VECTOR = getHubToSwerveVectorComponents()
        val xComponentMeters = HUB_TO_SWERVE_VECTOR.first.`in`(Meters)
        val yComponentMeters = HUB_TO_SWERVE_VECTOR.second.`in`(Meters)
        return hypot(xComponentMeters, yComponentMeters).meters
    }

    /**
     * Gets the HUB -> Swerve vector through [getHubToSwerveVectorComponents] and adds one third of the current
     * chassisSpeeds to account for the 20ms cycle delay. The new (x,y) components are then passed to [atan2] to
     * get the angle necessary for the Swerve to target the center point of the HUB.
     * @return The [Angle] at which the Swerve should be in order to target the center of the HUB.
     */
    fun getSwerveToHubAngle(): Angle {
        val HUB_TO_SWERVE_VECTOR = getHubToSwerveVectorComponents()

        val xAdjustedWithVelocity = HUB_TO_SWERVE_VECTOR.first.plus(drive.chassisSpeeds.vxMetersPerSecond.div(3).meters)
        val yAdjustedWithVelocity = HUB_TO_SWERVE_VECTOR.second.plus(drive.chassisSpeeds.vyMetersPerSecond.div(3).meters)

        return atan2(yAdjustedWithVelocity.`in`(Meters), xAdjustedWithVelocity.`in`(Meters)).radians
    }

    /**
     * Intended to run during Shooting State. Switches the [Hood] interpolation from distance driven to velocity
     * driven, allowing it to correct for [Shooter] RPMs losses.
     * @return A [RunCommand] interpolating the [Hood] angle based on the [Shooter] velocity.
     */
    fun shootingHoodInterpolation(): Command {
        return hood.setVelocityDrivenInterpolatedAngleCMD { shooter.getShooterAngularVelocity() }
    }

    /**
     * Intended to run before Shooting State. To prepare the [Hood], distance driven interpolation is performed.
     * This interpolation method must change during Shooting State to correct for any RPM losses.
     * @return A [RunCommand] interpolating the [Hood] angle based on the distance from Swerve to HUB.
     */
    fun preShootingHoodInterpolation(): Command {
        return hood.setDistanceDrivenInterpolatedAngleCMD { getSwerveToHubDistance() }
    }

    /**
     * Due to hopper's design, there is the possibility of FUELS getting stuck during Shooting State. To correct this,
     * the deployable component of the intake must go backwards and upwards in intervals of 15 - 30 degrees, pushing
     * FUELS towards the indexer and therefore, the [Shooter].
     * @return A [RunCommand] that makes the intake "dance" by moving it 15 degrees up and down, with its maximum
     *      being the intake's retracted angle.
     */
    fun intakeShootingDanceCMD(): Command {
        return RunCommand({
            intake.setDeployableAngleOnly(IntakeConstants.RetractileAngles.RetractedAngle
                .plus(RobotConstants.Control.INTAKE_DEPLOYABLE_DANCE_DELTA))
            WaitUntilCommand { intake.getDeployableError() < RobotConstants.Control.INTAKE_DEPLOYABLE_DANCE_TOLERANCE }
            intake.setDeployableAngleOnly(IntakeConstants.RetractileAngles.RetractedAngle)
            WaitUntilCommand { intake.getDeployableError() < RobotConstants.Control.INTAKE_DEPLOYABLE_DANCE_TOLERANCE }
        }, intake)
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