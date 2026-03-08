package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.Constants.isFlipped
import frc.robot.constants.FieldZones
import frc.robot.constants.RobotConstants
import frc.robot.utils.subsystemUtils.identification.SysIdRoutines
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
        configureStatesConditions()
        //configureStatesCommands()
        configureBindings()
    }

    @AutoLogOutput(key = RobotConstants.Telemetry.STATES_CURRENT_STATE_FIELD)
    private fun getCurrentState(): String {
        return stateMachine.getCurrentState().name
    }

    private fun configureStatesConditions() {
        controller.start().onTrue(superstructure.resetDrivePose()) // Reset rotation
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
            ShootState,
            Phase.Teleop,
            4
        )

        stateMachine.addCondition(
            { controller.rightBumper().asBoolean },
            IntakeState,
            Phase.Teleop,
            5
        )

        stateMachine.addCondition(
            { controller.x().asBoolean },
            GoingUnderTrench,
            Phase.Teleop,
            6
        )

        stateMachine.addCondition(
            { controller.povLeft().asBoolean } ,
            EmergencyShootState,
            Phase.Teleop,
            999
        )
    }

    private fun configureStatesCommands() {
//        AssistState.setInitialCommand(superstructure.driveTargetingBumpAssist())
//        AssistState.setDefaultCommand(superstructure.assistStateSequenceDefaultCMD())
//        AssistState.setEndCommand(superstructure.driveFollowingDriverInput().andThen(
//            superstructure.disableSubsystems()
//        ))
//
//        ShootState.setInitialCommand(superstructure.driveTargetingHUB())
//        ShootState.setDefaultCommand(superstructure.shootStateSequenceDefaultCMD())
//        ShootState.setEndCommand(superstructure.driveFollowingDriverInput().andThen(
//            superstructure.disableSubsystems()
//        ))
//
//        IntakeState.setDefaultCommand(superstructure.intakeStateCMD())
//        //IntakeState.setEndCommand(superstructure.disableIntake())
//
//        EmergencyShootState.setDefaultCommand(superstructure.shootStateSequenceWithoutOdometryCMD())
        //EmergencyShootState.setEndCommand(superstructure.disableSubsystems())

        //GoingUnderTrench.setInitialCommand(superstructure.storeHood())
//        GoingUnderTrench.setDefaultCommand(superstructure.followTrajectory(
//            superstructure.getOnTheFlyPathFromWaypoints(
//                superstructure.getTrenchOnTheFlyWaypointList(),
//                Degrees.zero()
//            )
//        ))
    }

    private fun configureBindings() {
        controller.start().onTrue(superstructure.resetDrivePose()) // Reset rotation

        controller.povUp().onTrue(superstructure.coastSubsystems().onlyIf { DriverStation.isDisabled() }
            .ignoringDisable(true))   // Coast Intake + Hood
        controller.povDown().onTrue(superstructure.brakeSubsystems().onlyIf { DriverStation.isDisabled() }
            .ignoringDisable(true)) // Brake Intake + Hood

        controller.povLeft().onTrue(superstructure.driveTargetingBumpAssist())
            .onFalse(superstructure.driveFollowingDriverInput())
        controller.povRight().onTrue(superstructure.driveTargetingHUB())
            .onFalse(superstructure.driveFollowingDriverInput())


        controller.rightTrigger().onTrue(superstructure.shootStateSequenceDefaultCMD())
            .onFalse(superstructure.disableSubsystems())

        controller.leftTrigger().onTrue(superstructure.assistStateSequenceDefaultCMD())
            .onFalse(superstructure.disableSubsystems())

        controller.y().onTrue(superstructure.noStateShootOnly())
            .onFalse(superstructure.disableShooter())

        controller.b().onTrue(superstructure.noStateIndexerOnly())
            .onFalse(superstructure.disableIndexer())

        controller.a().onTrue(superstructure.noStateHoodOnly())

        controller.rightBumper().onTrue(superstructure.intakeStateCMD())
            .onFalse(superstructure.disableIntake())

//        controller.x().whileTrue(superstructure.followTrajectory(
//            superstructure.getOnTheFlyPathFromWaypoints(
//                superstructure.getTrenchOnTheFlyWaypointList(), Degrees.zero()
//            )
//        ))
    }
}