package frc.robot.subsystems

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.constants.Constants.isFlipped
import frc.robot.constants.FieldZones
import frc.robot.constants.LedPatterns
import frc.robot.constants.RobotConstants
import net.tecdroid.util.stateMachine.Phase
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States.*
import org.littletonrobotics.junction.AutoLogOutput

class StatesHandler(
    private val superstructure: Superstructure,
    private val controller: CommandXboxController
) {

    private val stateMachine: StateMachine = StateMachine(NeutralState)

    init {
        //configureStatesConditions()
        //configureStatesCommands()
        configureBindings()
    }

    @AutoLogOutput(key = RobotConstants.Telemetry.STATES_CURRENT_STATE_FIELD)
    private fun getCurrentState(): String {
        return stateMachine.getCurrentState().name
    }

    private fun configureStatesConditions() {
        stateMachine.addCondition(
            { superstructure.isInsideZone(FieldZones.NEUTRAL_ZONE) },
            NeutralState,
            Phase.Teleop,
            1
        )

        stateMachine.addCondition(
            { superstructure.isInsideZone(
                if (isFlipped.invoke()) FieldZones.RED_ALLIANCE_ZONE else FieldZones.BLUE_ALLIANCE_ZONE
            ) },
            PrepShootingState,
            Phase.Teleop,
            2
        )

        stateMachine.addCondition(
            { superstructure.isInsideZone(FieldZones.NEUTRAL_ZONE) && controller.rightTrigger().asBoolean },
            AssistState,
            Phase.Teleop,
            3
        )

        stateMachine.addCondition(
            { superstructure.isInsideZone(
                if (isFlipped.invoke()) FieldZones.RED_ALLIANCE_ZONE else FieldZones.BLUE_ALLIANCE_ZONE
            ) && controller.rightTrigger().asBoolean },
            ScoreState,
            Phase.Teleop,
            4
        )

        stateMachine.addCondition(
            { controller.rightBumper().asBoolean },
            IntakeState,
            Phase.Teleop,
            5
        )

//        stateMachine.addCondition(
//            { controller.x().asBoolean },
//            GoingUnderTrench,
//            Phase.Teleop,
//            6
//        )

        stateMachine.addCondition(
            { controller.povLeft().asBoolean } ,
            EmergencyShootState,
            Phase.Teleop,
            999
        )
    }

    private fun configureStatesCommands() {
        AssistState.setInitialCommand(
            superstructure.driveTargetingBump().alongWith(
                superstructure.assistStateSequenceDefaultCMD()
            )
        )

        AssistState.setEndCommand(
            superstructure.driveFollowingDriverInput().andThen(
                superstructure.disableSubsystemsInitCMD()
            )
        )

        ScoreState.setInitialCommand(
            superstructure.driveTargetingHUB().alongWith(
                superstructure.scoreStateSequenceDefaultCMD()
            )
        )

        ScoreState.setEndCommand(
            superstructure.driveFollowingDriverInput().andThen(
                superstructure.disableSubsystemsInitCMD()
            )
        )

        IntakeState.setInitialCommand(superstructure.intakeStateCMD())
        IntakeState.setEndCommand(superstructure.disableIntakeRollersCMD())

        EmergencyShootState.setInitialCommand(superstructure.scoreStateSequenceWithoutOdometryCMD())
        EmergencyShootState.setEndCommand(superstructure.disableSubsystemsInitCMD())

        //GoingUnderTrench.setInitialCommand(superstructure.storeHood())
//        GoingUnderTrench.setDefaultCommand(superstructure.followTrajectory(
//            superstructure.getOnTheFlyPathFromWaypoints(
//                superstructure.getTrenchOnTheFlyWaypointList(),
//                Degrees.zero()
//            )
//        ))
    }

    private fun configureBindings() {
        controller.start().onTrue(Commands.select(
            mapOf<Alliance, Command>(
                Alliance.Blue to superstructure.resetDrivePoseBlueCMD(),
                Alliance.Red to superstructure.resetDrivePoseRedCMD()
            ), { if (isFlipped.invoke()) Alliance.Red else Alliance.Blue }
        )) // Reset rotation


        controller.rightTrigger(0.8).whileTrue(Commands.select(
            mapOf<FieldZones, Command>(
                FieldZones.BLUE_ALLIANCE_ZONE to superstructure.scoreStateSequenceDefaultCMD()
                    .alongWith(setShootingLed()),
                FieldZones.RED_ALLIANCE_ZONE to superstructure.scoreStateSequenceDefaultCMD()
                    .alongWith(setShootingLed()),
                FieldZones.NEUTRAL_ZONE to superstructure.assistStateSequenceDefaultCMD()
                    .alongWith(setShootingLed())
            ),
            { superstructure.getRobotCurrentZone() }
        ))
            .onFalse(superstructure.disableSubsystemsCMD().alongWith(setDefaultLed()))

        controller.y().whileTrue(Commands.select(
            mapOf<FieldZones, Command>(
                FieldZones.BLUE_ALLIANCE_ZONE to superstructure.driveTargetingHUB()
                    .onlyWhile { superstructure.isDriveAtScoreSetpoint().not() },
                FieldZones.RED_ALLIANCE_ZONE to superstructure.driveTargetingHUB()
                    .onlyWhile { superstructure.isDriveAtScoreSetpoint().not() },
                FieldZones.NEUTRAL_ZONE to superstructure.driveTargetingBump()
            ),
            { superstructure.getRobotCurrentZone() })
            .andThen(superstructure.stopDriveWithX()
                .alongWith(setDriveLockedLed())))
            .onFalse(
                superstructure.driveFollowingDriverInput()
                    .alongWith(setDefaultLed()))

        controller.leftBumper().onTrue(superstructure.scoreStateSequenceWithoutOdometryCMD()
            .alongWith(setShootingLed()))
            .onFalse(superstructure.disableSubsystemsCMD()
                .alongWith(setDefaultLed()))

        controller.rightBumper().onTrue(superstructure.intakeStateCMD()
            .alongWith(setIntakeLed()))
            .onFalse(superstructure.disableIntakeRollersCMD()
                .alongWith(setDefaultLed()))

//        controller.x().whileTrue(superstructure.followTrajectory(
//            superstructure.getOnTheFlyPathFromWaypoints(
//                superstructure.getTrenchOnTheFlyWaypointList(), Degrees.zero()
//            )
//        ))

        controller.povUp().onTrue(superstructure.coastSubsystems().onlyIf { DriverStation.isDisabled() }
            .ignoringDisable(true))   // Coast Intake + Hood
        controller.povDown().onTrue(superstructure.brakeSubsystems().onlyIf { DriverStation.isDisabled() }
            .ignoringDisable(true)) // Brake Intake + Hood
    }

    private fun configureRawBindings() {

        controller.y().onTrue(superstructure.noStateShootOnlyCMD()).onFalse(superstructure.disableShooterCMD())

        controller.b().onTrue(superstructure.noStateHopperBeltsOnly())
            .onFalse(superstructure.disableIndexerCMD())

        controller.x().whileTrue(superstructure.noStateFeederRollersOnly())
            .onFalse(superstructure.disableIndexerCMD())

        controller.a().whileTrue(superstructure.driveTrackingTarget()).onFalse(superstructure.driveFollowingDriverInput())

        //controller.a().whileTrue(superstructure.stopDriveWithX())

        //controller.y().onTrue(superstructure.noStateIntakeDeployableOnlyDisableCMD())
    }

    private fun configureLowInterpolationBindings() {
        // Low curvature interpolation

        controller.leftTrigger().onTrue(Commands.select(
            mapOf<FieldZones, Command>(
                FieldZones.BLUE_ALLIANCE_ZONE to superstructure.scoreStateLowCurvatureSequenceDefaultCMD()
                    .alongWith(setShootingLed()),
                FieldZones.RED_ALLIANCE_ZONE to superstructure.scoreStateLowCurvatureSequenceDefaultCMD()
                    .alongWith(setShootingLed()),
                FieldZones.NEUTRAL_ZONE to superstructure.assistStateSequenceDefaultCMD()
                    .alongWith(setShootingLed())
            ),
            { superstructure.getRobotCurrentZone() }
        ))
            .onFalse(superstructure.disableSubsystemsCMD()
                .alongWith(setDefaultLed()))
    }

    fun driverControllerRumbleBoth(): Command = RunCommand({ controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.5) })
    fun disableControllerRumble(): Command = InstantCommand({ controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)})

    fun setDefaultLed(): Command {
        return superstructure.setLEDPattern(LedPatterns.SolidColors.GOLD)
    }

    fun setShootingLed(): Command {
        return superstructure.setLEDPattern(LedPatterns.FixedPalettePatters.STROBE_BLUE)
    }

    fun setIntakeLed(): Command {
        return superstructure.setLEDPattern(LedPatterns.SolidColors.BLUE_GREEN)
    }

    fun setDriveLockedLed(): Command {
        return superstructure.setLEDPattern(LedPatterns.SolidColors.VIOLET)
    }

    fun setAutonomousLed(): Command {
        return superstructure.setLEDPattern(LedPatterns.ColorOnePatterns.BREATH_FAST)
    }
 }