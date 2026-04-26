package frc.robot.subsystems

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.Waypoint
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
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
import frc.robot.constants.Pattern
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
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

class Superstructure(private val controller: CommandXboxController) : Subsystem {
    /*-------------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- SUBSYSTEM DECLARATION ----------------------------------------*/
    /*-------------------------------------------------------------------------------------------------------*/

    // IMPORTANT: Keep Module other as follows: FL, FR, BL, Br. Otherwise, kinematics will fail.
    private val drive   : Drive     = Drive(
        GyroIOPigeon2(),
        ModuleIOTalonFX(SwerveTunerConstants.FrontLeft),
        ModuleIOTalonFX(SwerveTunerConstants.FrontRight),
        ModuleIOTalonFX(SwerveTunerConstants.BackLeft),
        ModuleIOTalonFX(SwerveTunerConstants.BackRight)
    )

    private val intake  : Intake        = Intake()
    private val indexer : Indexer       = Indexer()
    private val shooter : Shooter       = Shooter()
    private val hood    : Hood          = Hood()
    private val leds    : LedController = LedController()

    /*--------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- USEFUL VARIABLES ----------------------------------------*/
    /*--------------------------------------------------------------------------------------------------*/

    // For the robot to remember its target angle once the driver releases right joystick
    private val lastDriveAngle: MutAngle = Radians.mutable(drive.rotation.radians)

    /*-------------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- COMPLEMENTARY SYSTEMS ----------------------------------------*/
    /*-------------------------------------------------------------------------------------------------------*/

    // For odometry corrections. Order of cameras is not relevant; all of them require the Drive's rotation.
    private val vision = Vision(
        drive::addVisionMeasurement,
        VisionIOLimelight(VisionConstants.leftLimelight,   { drive.rotation }),
        VisionIOLimelight(VisionConstants.backLimelight,    { drive.rotation }),
        VisionIOLimelight(VisionConstants.rightLimelight,   { drive.rotation })
    )

    // TESTING. Intended to perform all shooting-related calculations, including Shooter RPS and Hood and Drive angles.
    private val shootingCalcs: ShootingCalcs = ShootingCalcs(
        { if (isInsideZone(FieldZones.NEUTRAL_ZONE)) getBumpTranslation() else getHubTranslation() }, // Target Translation
        { drive.pose },
        { drive.robotRelativeSpeeds },
        { drive.fieldRelativeSpeeds },
        { hood.getHoodAngleDegrees().degrees },
        { isInsideZone(FieldZones.NEUTRAL_ZONE) },
    )

    /*--------------------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- AUTO, TELEOP & DISABLED INIT ----------------------------------------*/
    /*--------------------------------------------------------------------------------------------------------------*/

    /**
     * Intended for Auto Init. Prevents the [Drive]'s default command to interfere with path following commands.
     */
    fun removeDriveDefaultCommand() {
        drive.removeDefaultCommand()
    }

    /**
     * For autonomous routine selection.
     */
    fun getAutoChooser(): SendableChooser<Command> = drive.autoChooser

    /**
     * Intended for Teleop Init. Gives the [Drive]'s control to the driver as per the selected command.
     * @param command The default command for the driver. Normally [driveFollowingDriverInput].
     */
    fun setDriveDefaultCommand(command: Command) {
        drive.defaultCommand = command
    }

    /**
     * Intended for Disabled & Teleop Init. Cools down cameras by skipping the specified amount of frames.
     * @param throttle The amount of frames the camera should skip. The greater the number the more the frames the
     * camera skips. value between 0 - 250.
     */
    fun setVisionThrottle(throttle: Double) {
        vision.setThrottle(throttle)
    }

    /*---------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- SCORING SEQUENCES ----------------------------------------*/
    /*---------------------------------------------------------------------------------------------------*/

    /**
     * Intended for testing [shootingCalcs]. Follows same commands as [scoreStateSequenceDefaultCMD] with the difference
     * that [Drive] angle is controlled here and both [Shooter] and [Hood] targets are calculated through [shootingCalcs].
     * @return A [ParallelCommandGroup] with the above specifications.
     */
    fun alternateScoreSequence(): Command {
        return ParallelCommandGroup(
            driveTrackingTarget(), // Drive angle is no longer controlled by Driver, since it can be on the move.
            shooter.setVelocityCMD { shootingCalcs.getLatestShootingParameters().shooterRPS }, // Set calculated RPS
            hood.setAngleTunableCMD { shootingCalcs.getLatestShootingParameters().hoodAngle }, // Set calculated Angle
            WaitUntilCommand { shooter.getShooterAngularVelocityError() // Waits until required velocity is reached
                .lte(RobotConstants.Control.SHOOTER_VELOCITY_TOLERANCE)
                    // Waits until Drive angle is within tolerance
                    && getDriveCurrentRotationError() < RobotConstants.Control.DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING
            }.withTimeout(1.0.seconds)
                .andThen(indexerEnableCMD()) // Enables the Indexer, both hopper and tower rollers
                .andThen(WaitCommand(RobotConstants.Control.TIME_DELTA_BEFORE_CLUSTERING)
                    .andThen(intakeClusterCMD())) // Enables Intake clustering to push FUELS towards the tower
        )
    }

    /**
     * Intended for Score State. Starts by enabling the [Shooter] and [Hood] interpolation, waiting until the [Shooter]
     * RPS and [Drive] angle are within tolerance, then enabling the [Indexer]
     * and finally enabling the [Intake] clustering for faster shooting.
     * @return A [ParallelCommandGroup] with the above specifications.
     */
    fun scoreStateSequenceDefaultCMD(): Command {
        return ParallelCommandGroup(
            scoreStateShooterInterpolationCMD(), // Enables Shooter interpolation
            scoreStateHoodInterpolationCMD(), // Enables Hood interpolation
            WaitUntilCommand { shooter.getShooterAngularVelocityError() // Waits until required velocity is reached
                .lte(RobotConstants.Control.SHOOTER_VELOCITY_TOLERANCE)
            }.withTimeout(2.0.seconds)
                .andThen(indexerEnableCMD()) // Enables the Indexer, both hopper and tower rollers
                .andThen(WaitCommand(RobotConstants.Control.TIME_DELTA_BEFORE_CLUSTERING)
                    .andThen(intakeClusterCMD())) // Enables Intake clustering to push FUELS towards the tower
        )
    }

    /**
     * Intended to call during Auto. Same as [scoreStateSequenceDefaultCMD], just without checking [Drive] angle,
     * as it is controlled by PathPlanner.
     * - A little [WaitCommand] is used to ensure drive has the right rotation.
     * @return A [ParallelCommandGroup] with the above specifications.
     */
    fun scoreStateSequenceAutoRightCMD(): Command {
        return ParallelCommandGroup(
            WaitCommand(1.0.seconds), // Quick timeout for drive to target the HUB, as per PathPlanner path
            scoreStateShooterInterpolationCMD(),
            scoreStateHoodInterpolationCMD(),
            WaitUntilCommand { shooter.getShooterAngularVelocityError() // Waits until required velocity is reached
                .lte(RobotConstants.Control.SHOOTER_VELOCITY_TOLERANCE)
            }.withTimeout(1.2.seconds)
                .andThen(indexerEnableCMD()) // Enables the Indexer, both hopper and tower rollers
                .andThen(WaitCommand(RobotConstants.Control.TIME_DELTA_BEFORE_CLUSTERING)
                    .andThen(intakeClusterCMD())) // Enables Intake clustering to push FUELS towards the tower
        )
    }

    /**
     * Method intended for launching game pieces in case odometry fails its purpose.
     * Thus, the [driveTargetingHUB] or [driveTargetingBump] method won't be called, and it relies
     * entirely on the driver's aligning to the desired target.
     * [shooter] and [hood] respective interpolations won't be called either, a fixed shooter RPMs and hood
     * Angle is called instead.
     * @return A [ParallelCommandGroup] with the above specifications
     */
    fun scoreStateSequenceWithoutOdometryCMD(): Command {
        return ParallelCommandGroup(
            noStateShootOnlyCMD(), // Enables Shooter manual control
            noStateHoodOnlyCMD(), // Enables Hood manual control
            WaitUntilCommand { shooter.getShooterAngularVelocityError() // Waits until required velocity is reached
                .lte(RobotConstants.Control.SHOOTER_VELOCITY_TOLERANCE)
            }.withTimeout(2.0.seconds)
                .andThen(indexerEnableCMD()) // Enables the Indexer, both hopper and tower rollers
                .andThen(WaitCommand(RobotConstants.Control.TIME_DELTA_BEFORE_CLUSTERING)) // Safety for deployable assuming full hopper
                    .andThen(intakeClusterCMD())) // Enables Intake clustering to push FUELS towards the tower
    }

    /*----------------------------------------------------------------------------------------------------*/
    /*----------------------------------------- ASSIST SEQUENCES -----------------------------------------*/
    /*----------------------------------------------------------------------------------------------------*/

    /**
     * Same as [scoreStateSequenceDefaultCMD]. However, the target tracked is now the bump and the interpolation
     * works towards it. [assistStateShooterInterpolationCMD] and [assistStateHoodInterpolationCMD] methods are
     * now used to assertively set the hood's angle and shooter's RPMs
     * @return a [ParallelCommandGroup] with the above specifications
     */
    fun assistStateSequenceDefaultCMD(): Command {
        return ParallelCommandGroup(
            assistStateShooterInterpolationCMD(), // Enables Shooter interpolation
            assistStateHoodInterpolationCMD(), // Enables Hood interpolation
            WaitUntilCommand { shooter.getShooterAngularVelocityError() // Waits until required velocity is reached
                .lte(RobotConstants.Control.SHOOTER_VELOCITY_TOLERANCE)
            }.withTimeout(2.0.seconds)
                .andThen(indexerEnableCMD()) // Enables the Indexer, both hopper and tower rollers
                .andThen(WaitCommand(RobotConstants.Control.TIME_DELTA_BEFORE_CLUSTERING) // Safety for deployable assuming full hopper
                    .andThen(intakeClusterCMD()))) // Enables Intake clustering to push FUELS towards the tower
    }

    /*--------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- SCORING COMMANDS ----------------------------------------*/
    /*--------------------------------------------------------------------------------------------------*/

    /**
     * Intended to run during Score State. Enables the [Shooter] interpolation with respect to the HUB.
     * @return A [RunCommand] interpolating the [Shooter] velocity based on [getDriveToHubDistance].
     */
    private fun scoreStateShooterInterpolationCMD(): Command {
        return shooter.setScoreInterpolatedVelocityCMD { getDriveToHubDistance() }
    }

    /**
     * Intended to run during Score State. Enables the [Hood] interpolation with respect to the HUB.
     * @return A [RunCommand] interpolating the [Hood] angle based on [getDriveToHubDistance].
     */
    private fun scoreStateHoodInterpolationCMD(): Command {
        return hood.setScoreDistanceInterpolatedAngleCMD { getDriveToHubDistance() }
    }

    private fun scoreStateHoodLowCurvatureInterpolationCMD(): Command {
        return hood.setLowCurvatureScoreDistanceInterpolatedAngleCMD { getDriveToHubDistance() }
    }

    /*-------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- ASSIST COMMANDS ----------------------------------------*/
    /*-------------------------------------------------------------------------------------------------*/

    /**
     * Intended to run during [net.tecdroid.util.stateMachine.States.AssistState]. Switches the [Hood]
     * interpolation to, instead of getting the distance to the HUB, gets it from the Bump by calling
     * [getDriveToBumpDistance]
     */
    private fun assistStateShooterInterpolationCMD(): Command {
        return shooter.setAssistInterpolatedVelocity { getDriveToBumpDistance() }
    }

    /**
     * Intended to run during [AssistState]. Enables the [Hood] interpolation with respect to the target Bump.
     * @return A [RunCommand] interpolating the [Hood] angle based on [getDriveToBumpDistance].
     */
    private fun assistStateHoodInterpolationCMD(): Command {
        return hood.setAssistDistanceInterpolatedAngleCMD { getDriveToBumpDistance() }
    }

    /*-------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- INTAKE COMMANDS ----------------------------------------*/
    /*-------------------------------------------------------------------------------------------------*/

    /**
     * Method intended to use during [net.tecdroid.util.stateMachine.States.IntakeState] to deploy our robot's
     * intake. After that, the intakés displacement is checked and when a certain tolerance is met,
     * the rollers are enabled.
     * @return A [SequentialCommandGroup] with the above specifica
     */
    fun intakeStateCMD(): Command {
        return intake.deployAndEnableIntakeCMD()
    }

    /**
     * In the interest of shooting faster, the [Intake] retracts, causing the hopper to cluster the FUELS towards the tower.
     * @return A [SequentialCommandGroup] requesting the [Intake] to retract, and then deploying it again.
     */
    fun intakeClusterCMD(): Command {
        return SequentialCommandGroup(
            intake.clusterIntakeCMD(),
//            WaitUntilCommand { intake.getDeployableError().lte(IntakeConstants.RetractileAngles.DeployableDisplacementDelta)},
//            WaitCommand(RobotConstants.Control.TIME_DELTA_BEFORE_CLUSTER_END),
//            intake.deployAndDisableIntakeCMD()
        )
    }

    /*--------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- INDEXER COMMANDS ----------------------------------------*/
    /*--------------------------------------------------------------------------------------------------*/

    /**
     * Enables both the hopper and tower rollers.
     * @return A [ParallelCommandGroup] enabling hopper and tower rollers.
     */
    fun indexerEnableCMD(): Command {
        return indexer.enableIndexerCMD()
    }

    fun indexerIdleEnableCMD(): Command {
        return indexer.enableIndexerIdleCMD()
    }

    /*--------------------------------------------------------------------------------------------------*/
    /*---------------------------------------- LED COMMANDS --------------------------------------------*/
    /*--------------------------------------------------------------------------------------------------*/

    /**
     * Sets an LED Pattern
     */
    fun setLEDPattern(pattern: Pattern): Command {
        return leds.setPatternCMD(pattern)
    }

    /*-------------------------------------------------------------------------------------------------*/
    /*--------------------------------------- NO STATE COMMANDS ---------------------------------------*/
    /*-------------------------------------------------------------------------------------------------*/

    /**
     * Enables the intake rollers with the specified [IntakeConstants.VoltageTargets.ClusteringRollersVoltage]
     * which can be modified live for tuning. Intake's not deployed nor clustered. just the rollers enabled.
     * @return a [Command] with the above specifications
     */
    fun noStateIntakeRollersVoltageOnlyTunableCMD(): Command {
        return intake.setRollersVoltageTunableCMD { IntakeConstants.VoltageTargets.EnabledRollersVoltage }
    }

    fun noStateIntakeRollersOnlyVoltageEnableCMD(): Command {
        return intake.setRollersVoltageCMD()
    }

    fun noStateIntakeRollersOnlyVoltageDisableCMD(): Command {
        return intake.stopRollersVoltageCMD()
    }

    /**
     * Deploys our robot's intake, rollers are disabled as this method's purpose is to use ONLY the
     * retractile component of our intake, useful when testing.
     * @return A [Command] with the above specifications
     */
    fun noStateIntakeDeployableOnlyEnableCMD(): Command {
        return intake.setDeployableDisplacementOnly(IntakeConstants.RetractileAngles.DeployedDisplacement)
    }

    /**
     * Retracts our robot's intake, rollers are disabled as this method's purpose is to use ONLY the
     * retractile component of our intake, useful when testing.
     * @return A [Command] with the above specifications
     */
    fun noStateIntakeDeployableOnlyDisableCMD(): Command {
        return intake.setDeployableDisplacementOnly(IntakeConstants.RetractileAngles.ClusteredDisplacement)
    }

    fun noStateHopperBeltsOnly(): Command {
        return indexer.enableHopperBeltsCMD()
    }

    fun noStateFeederRollersOnly(): Command {
        return indexer.enableTowerRollersCMD()
    }

    /**
     * Intended to use during [net.tecdroid.util.stateMachine.States.EmergencyShootState].
     * A fixed RPMs are set to the [Shooter] without calling any interpolation as during the mentioned state,
     * the odometry is known to have failed.
     * @return A [Command] with the above specifications
     */
    fun noStateShootOnlyCMD(): Command {
        return shooter.setVelocityCMD { ShooterConstants.Tunables.enabledRPMs.get().div(60.0).rotationsPerSecond }
    }

    fun noStateShootOnlyCMD(rpms: Double): Command {
        return shooter.setVelocityCMD { rpms.div(60.0).rotationsPerSecond }
    }

    /**
     * Intended to use during [net.tecdroid.util.stateMachine.States.EmergencyShootState].
     * A fixed angle is set to the [Hood] without calling any interpolation as during the mentioned state,
     * the odometry is known to have failed.
     * @return A [Command] with the above specifications
     */
    fun noStateHoodOnlyCMD(): Command {
        return hood.setAngleTunableCMD { HoodConstants.Tunables.hoodTunableAngle.get().degrees }
    }

    /*----------------------------------------------------------------------------------------------------------*/
    /*--------------------------------------- SUBSYSTEM DISABLE COMMANDS ---------------------------------------*/
    /*----------------------------------------------------------------------------------------------------------*/

    /**
     * Requests the [Hood] lowest position. This ensures it will not be damaged when going below the trench.
     * @return An [InstantCommand] requesting the [Hood] lowest position.
     */
    fun storeHoodCMD(): Command {
        return hood.setAngleCMD(HoodConstants.Control.HoodLowestPosition)
    }

    /**
     * Disables the ROLLERS of the [Intake]. Deployable component does NOT receive any request.
     * @return An [InstantCommand] disabling the [Intake] rollers.
     */
    fun disableIntakeRollersCMD(): Command {
        return intake.disableRollersCMD()
    }

    /**
     * Disables the [Indexer] belts. Useful ONLY when robot' not shooting.
     * @return  A [ParallelCommandGroup] which stops every indexing motor.
     */
    fun disableIndexerCMD(): Command {
        return indexer.stopIndexerCMD()
    }

    /**
     * Stops the [Shooter] rollers.
     * @return An [InstantCommand] stoping the [Shooter] rollers.
     */
    fun disableShooterCMD(): Command {
        return shooter.stopShooterCMD()
    }

    /**
     * Intended to use any time the driver is done shooting. This method stores the [Hood], disables the [Indexer],
     * [Shooter] and [Intake] rollers while commanding deployed position.
     * @return A [ParallelCommandGroup] with the above requests.
     */
    fun disableSubsystemsCMD(): Command {
        return ParallelCommandGroup(
            storeHoodCMD(),
            SequentialCommandGroup(
                intake.stopMotor(),
                intake.deployAndDisableIntakeCMD()
            ),
            disableShooterCMD(),
            disableIndexerCMD()
        )
    }

    /**
     * Intended to use any time the driver is done shooting. This method stores the [Hood], disables the [Indexer],
     * [Shooter] and [Intake] rollers. [Intake] is NOT deployed again.
     * @return A [ParallelCommandGroup] with the above requests.
     */
    fun disableSubsystemsAutoCMD(): Command {
        return ParallelCommandGroup(
            storeHoodCMD(),
            SequentialCommandGroup(
                intake.stopMotor(),
                disableIntakeRollersCMD()
            ),
            disableShooterCMD(),
            disableIndexerCMD()
        )
    }

    /**
     * Intended to use any time the driver is done shooting. This method stores the [Hood], disables the [Indexer],
     * [Shooter] and [Intake] ROLLERS. This method does NOT request an [Intake] position.
     * @return A [ParallelCommandGroup] with the above requests.
     */
    fun disableSubsystemsInitCMD(): Command {
        return ParallelCommandGroup(
            storeHoodCMD(),
            disableIndexerCMD(),
            disableIntakeRollersCMD(),
            disableShooterCMD()
        )
    }

    /*--------------------------------------------------------------------------------------------------*/
    /*----------------------------------------- DRIVE COMMANDS -----------------------------------------*/
    /*--------------------------------------------------------------------------------------------------*/

    /**
     * Intended to test [shootingCalcs]. Same as [driveTargetingHUB] and [driveTargetingBump], with the difference
     * that translation max velocity is always limited and the target angle is obtained through [shootingCalcs].
     * Once the [Drive] is within tolerance, it will stop with an X-arrangement, making it virtually impossible for
     * other robots to move it.
     * TODO() = Check stopWithX behaviour. Might use a RunCommand that checks whether in tolerance or not if this freezes.
     * @return A [RunCommand] that locks the [Drive] angle to track the target.
     */
    fun driveTrackingTarget(): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { -controller.leftY * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_Y_MULTIPLIER },
            { -controller.leftX * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_X_MULTIPLIER },
            { shootingCalcs.getLatestShootingParameters().driveAngle }
        )//.until { getDriveRotationError().lte(RobotConstants.Control.DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING) }
//            .andThen(InstantCommand({ drive.stopWithX() }))
    }

    /**
     * Intended to be Initial Command of Score State. Locks the [Drive] angle to track the HUB's coordinates.
     * During this command the driver won't be able to rotate the chassis, however, the translation is still under
     * his control. Translation max velocity is limited during this command.
     * @return A [RunCommand] that locks the [Drive] angle to target the HUB.
     */
    fun driveTargetingHUB(): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { -controller.leftY * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_Y_MULTIPLIER },
            { -controller.leftX * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_X_MULTIPLIER },
            ::getDriveToHubAngle
        )
    }

    /**
     * Intended to be Initial Command of Score State. Locks the [Drive] angle to track the HUB's coordinates.
     * During this command the driver won't be able to rotate the chassis, however, the translation is still under
     * his control. Translation max velocity is limited during this command.
     * @return A [RunCommand] that locks the [Drive] angle to target the HUB.
     */
    fun driveTargetingHUBAuto(): Command {
        return DriveCommands.joystickDriveAtAngleAuto(
            drive,
            { -controller.leftY * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_Y_MULTIPLIER },
            { -controller.leftX * RobotConstants.DriverControllerConstants.SWERVE_LOCKED_ANGLE_X_MULTIPLIER },
            ::getDriveToHubAngle
        )
    }

    /**
     * Intended to be part of the Initial Command of [net.tecdroid.util.stateMachine.States.AssistState].
     * Locks the [Drive] angle to track the Bump's coordinates. During this command the driver won't be able to
     * rotate the chassis, however, the translation is still under his control. No limitation in translation is applied.
     * @return A [RunCommand] that locks the [Drive] angle to target the Bump.
     */
    fun driveTargetingBump(): Command {
        return DriveCommands.joystickDriveAtAngle(
            drive,
            { -controller.leftY * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_Y_MULTIPLIER },
            { -controller.leftX * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_X_MULTIPLIER },
            ::getDriveToBumpAngle
        )
    }

    /**
     * Intended to be [Drive] default command & End Command of Score State. Gives the full [Drive] control to the driver.
     * @return A [RunCommand] with the [Drive]'s default command.
     */
    fun driveFollowingDriverInput(): Command {
        return DriveCommands.joystickDriveAtAngle (
            drive,
            { MathUtil.applyDeadband(-controller.leftY, 0.05) * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_Y_MULTIPLIER },
            { MathUtil.applyDeadband(-controller.leftX, 0.05) * RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_X_MULTIPLIER },
            { getAngleFromJoystick() }
        )
    }

    fun alignChassisToHubAuto(): Command {
        return DriveCommands.alignDriveAtAngle(drive, getDriveToHubAngle())
    }

    /**
     * Stops the [Drive] with an X arrangement in the wheels to avoid movement.
     * @return A [RunCommand] with the above specifications.
     */
    fun stopDriveWithX(): Command {
        return RunCommand({ drive.stopWithX() }, drive)
    }

    /**
     * Intended to reset [drive.pose] when in Red Alliance. This means, keep translation and override angle to 180.
     * @return An [InstantCommand] resetting the [Drive] pose.
     */
    fun resetDrivePoseRedCMD(): Command {
        return InstantCommand({ drive.pose = Pose2d(drive.pose.translation, Rotation2d.k180deg) }, drive)
    }

    /**
     * Intended to reset [drive.pose] when in Blue Alliance. This means, keep translation and override angle to 0.
     * @return An [InstantCommand] resetting the [Drive] pose.
     */
    fun resetDrivePoseBlueCMD(): Command {
        return InstantCommand({ drive.pose = Pose2d(drive.pose.translation, Rotation2d.kZero) }, drive)
    }

    /**
     * Intended to reset [drive.pose] before following a single [PathPlannerPath].
     * @param pose2d The desired [Pose2d] to set the odometry to.
     * @return An [InstantCommand] resetting the [Drive] pose.
     */
    fun resetDrivePoseCMD(pose2d: Pose2d): Command {
        return InstantCommand({ drive.pose = pose2d }, drive)
    }

    /**
     * Commands the [Drive] to follow the provided [PathPlannerPath] through [com.pathplanner.lib.auto.AutoBuilder].
     * @param path The desired [PathPlannerPath] to follow.
     * @return An [com.pathplanner.lib.auto.AutoBuilder] command to follow the specified path.
     */
    fun followTrajectoryCMD(path: PathPlannerPath): Command {
        return drive.followTrajectory(path)
    }

    /*--------------------------------------------------------------------------------------------------------*/
    /*----------------------------------------- DRIVE HELPER METHODS -----------------------------------------*/
    /*--------------------------------------------------------------------------------------------------------*/

    /**
     * Gets the desired [Drive] angle based on the [controller] right joystick. Adjustments for alliance and
     * joystick releasing are performed.
     * @return The desired [Drive] angle based on Drivers use of the [controller] right joystick.
     */
    @AutoLogOutput(key = "Odometry/Joystick Target Angle")
    private fun getAngleFromJoystick(): Angle {
        if (abs(-controller.rightY) < 0.3 && abs(controller.rightX) < 0.3) return lastDriveAngle

        val atan2Rad = atan2( // Gets the "raw" angle from the joysticks, after applying a dead band
            MathUtil.applyDeadband(
                if (isFlipped.invoke()) -controller.rightY else controller.rightY, 0.3),
            MathUtil.applyDeadband(
                if (isFlipped.invoke()) controller.rightX else -controller.rightX, 0.3)
        ).plus(Math.PI / 2)

        lastDriveAngle.mut_replace(atan2Rad.radians)

        // To ensure consistent rotation, 180.0 degrees are added when in Blue Alliance.
        return lastDriveAngle
    }

    /**
     * The difference between the target angle (updated in [lastDriveAngle]) and the current [Drive] angle.
     * @return The [Drive] error to reach target angle.
     */
    fun getDriveCurrentRotationError(): Angle {
        return abs(lastDriveAngle.`in`(Degrees) - drive.rotation.degrees).degrees
    }

    /**
     * The difference between the HUB angle and the current [Drive] angle.
     * @return The [Drive] error to reach HUB angle.
     */
    @AutoLogOutput(key = "Odometry/Drive to HUB Theta Error")
    fun getDriveToHubRotationError(): Angle {
        return abs(getDriveToHubTargetAngle().`in`(Degrees) - drive.rotation.degrees).degrees
    }

    /**
     * Determines whether the [Drive] is within tolerance to score into the HUB.
     * The .minus(2 * PI) is because in the upper side of the field, [getDriveToHubRotationError]
     * returns the error angle plus one rotation.
     * @return Whether the [Drive] is within tolerance to score into the HUB.
     */
    @AutoLogOutput(key = "Odometry/IsDriveAtSetpoint")
    fun isDriveAtScoreSetpoint(): Boolean {
        val rotationError = if (getDriveToHubRotationError().gte(Math.PI.times(2.0).radians))
            getDriveToHubRotationError().minus(Math.PI.times(2.0).radians) else getDriveToHubRotationError()
        return abs(rotationError.`in`(Degrees)) <
                (RobotConstants.Control.DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING.`in`(Degrees))
    }

    /**
     * Gets the [Drive] -> HUB vector and calculates its angle. PI is added to account for [Shooter] facing backwards.
     * Unlike [getDriveToHubAngle], this method does not update [lastDriveAngle], since it is only used to determine
     * if the [Drive] is at setpoint when scoring.
     * @return The [Drive] -> HUB [Angle]
     */
    private fun getDriveToHubTargetAngle(): Angle {
        val driveToHubTranslation = getDriveToHubTranslation()

        // Shooter is facing backwards with reference to the robot, hence we add PI to shoot.
        val atan2Rad = atan2(driveToHubTranslation.measureY.`in`(Meters), driveToHubTranslation.measureX.`in`(Meters))
            .plus(Math.PI).radians

        return atan2Rad
    }

    /**
     * Used for Telemetry purposes.
     * @return The value contained in [lastDriveAngle]
     */
    @AutoLogOutput(key = "Odometry/Last Registered Angle")
    private fun getLastAngle(): Angle {
        return lastDriveAngle
    }

    /**
     * Gets the [Drive] -> HUB vector and calculates its magnitude.
     * - Think of it as a right triangle, then (x,y) are the sides and the hypotenuse is the vector's magnitude.
     * @return The [Drive] -> HUB [Distance]
     */
    @AutoLogOutput(key = "Odometry/Swerve Distance to HUB", unit = "meters")
    private fun getDriveToHubDistance(): Distance {
        val driveToHubTranslation = getDriveToHubTranslation()
        return hypot(driveToHubTranslation.measureX.`in`(Meters), driveToHubTranslation.measureY.`in`(Meters)).meters
    }

    /**
     * Gets the [Drive] -> Target Bump vector and calculates its magnitude.
     * - Think of it as a right triangle, then (x,y) are the sides and the hypotenuse is the vector's magnitude.
     * @return The [Drive] -> Target Bump [Distance]
     */
    @AutoLogOutput(key = "Odometry/Swerve Distance to Bump Assist", unit = "meters")
    private fun getDriveToBumpDistance(): Distance {
        val driveToBumpTranslation = getDriveToBumpTranslation()
        return hypot(driveToBumpTranslation.measureX.`in`(Meters), driveToBumpTranslation.measureY.`in`(Meters)).meters
    }

    /**
     * Gets the [Drive] -> HUB vector and calculates its angle. PI is added to account for [Shooter] facing backwards.
     * @return The [Drive] -> HUB [Angle]
     */
    private fun getDriveToHubAngle(): Angle {
        val driveToHubTranslation = getDriveToHubTranslation()

        // Shooter is facing backwards with reference to the robot, hence we add PI to shoot.
        val atan2Rad = atan2(driveToHubTranslation.measureY.`in`(Meters), driveToHubTranslation.measureX.`in`(Meters))
            .plus(Math.PI).radians
        lastDriveAngle.mut_replace(atan2Rad)

        return lastDriveAngle
    }

    /**
     * Gets the [Drive] -> Target Bump vector and calculates its angle. PI is added to account for [Shooter] facing backwards.
     * @return The [Drive] -> Target Bump [Angle]
     */
    private fun getDriveToBumpAngle(): Angle {
        val driveToBumpTranslation = getDriveToBumpTranslation()

        // Shooter is facing backwards with reference to the robot, hence we add PI to shoot.
        val atan2Rad = atan2(driveToBumpTranslation.measureY.`in`(Meters), driveToBumpTranslation.measureX.`in`(Meters))
            .plus(Math.PI).radians
        lastDriveAngle.mut_replace(atan2Rad)

        return lastDriveAngle
    }

    // TODO() = Either check the methods below for on-the-fly paths or delete them.

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
            Pose2d(if (isFlipped.invoke()) FieldConstants.Trench.RED_TRENCH_CENTER_X else FieldConstants.Trench.BLUE_TRENCH_CENTER_X,
            yCoordinate, Rotation2d.kZero),
            Pose2d(xCoordinate, yCoordinate, Rotation2d.kZero)
        )
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /*--------------------------------------- NEUTRAL MODE COMMANDS ---------------------------------------*/
    /*-----------------------------------------------------------------------------------------------------*/

    /**
     * Sends a coast request to the [Intake] and [Hood]
     */
    fun coastSubsystems(): Command {
        return ParallelCommandGroup(
            intake.coastCMD(),
            hood.coastCMD()
        )
    }

    /**
     * Sends a brake request to the [Intake] and [Hood]
     */
    fun brakeSubsystems(): Command {
        return ParallelCommandGroup(
            intake.brakeCMD(),
            hood.brakeCMD()
        )
    }

    /*----------------------------------------------------------------------------------------------------*/
    /*--------------------------------------- FIELD HELPER METHODS ---------------------------------------*/
    /*----------------------------------------------------------------------------------------------------*/

    /**
     * Constructs a [Translation2d] containing the coordinates of the target HUB, after accounting for alliance.
     * @return The alliance HUB [Translation2d]
     */
    private fun getHubTranslation(): Translation2d {
        val centerHubX = if (isFlipped.invoke()) FieldConstants.HUB.RED_HUB_CENTER_X else FieldConstants.HUB.BLUE_HUB_CENTER_X
        val centerHubY = if (isFlipped.invoke()) FieldConstants.HUB.RED_HUB_CENTER_Y else FieldConstants.HUB.BLUE_HUB_CENTER_Y

        return Translation2d(centerHubX, centerHubY)
    }

    /**
     * Constructs a [Translation2d] containing the coordinates of the target Bump, after accounting for alliance and
     * field position.
     * @return The alliance target Bump [Translation2d]
     */
    private fun getBumpTranslation(): Translation2d {
        val centerBumpX = if (isFlipped.invoke()) FieldConstants.Bump.RED_BUMP_CENTER_X else FieldConstants.Bump.BLUE_BUMP_CENTER_X
        val centerBumpY = if (drive.pose.measureY.gt(FieldConstants.Dimensions.FIELD_WIDTH_Y.div(2.0)))
            FieldConstants.Bump.UPPER_BUMP_Y else FieldConstants.Bump.LOWER_BUMP_Y

        return Translation2d(centerBumpX, centerBumpY)
    }

    /** Constructs a [Translation2d] representing the vector [Drive] -> HUB.
     * @return The [Drive] -> HUB [Translation2d]
     */
    private fun getDriveToHubTranslation(): Translation2d {
        val hubTranslation = getHubTranslation()
        val drivePose = drive.pose

        return Translation2d(
            hubTranslation.measureX.minus(drivePose.measureX),
            hubTranslation.measureY.minus(drivePose.measureY)
        )
    }

    /** Constructs a [Translation2d] representing the vector [Drive] -> Target Bump.
     * @return The [Drive] -> Target Bump [Translation2d]
     */
    private fun getDriveToBumpTranslation(): Translation2d {
        val bumpTranslation = getBumpTranslation()
        val drivePose = drive.pose

        return Translation2d(
            bumpTranslation.measureX.minus(drivePose.measureX),
            bumpTranslation.measureY.minus(drivePose.measureY)
        )
    }

    /**
     * Helper method to check whether the robot is in a specified field zone.
     * @param zone The desired [FieldZones] to check if the robot is in.
     * @return True if the robot is in the specified zone, false otherwise.
     */
    fun isInsideZone(zone: FieldZones): Boolean {
        return getRobotCurrentZone() == zone
    }

    /**
     * Checks the x-component of [drive.pose] to determine in which [FieldZones] the robot is in.
     * @return The current [FieldZones] of the robot.
     */
    @AutoLogOutput(key = "Odometry/Current Zone")
    fun getRobotCurrentZone(): FieldZones {
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