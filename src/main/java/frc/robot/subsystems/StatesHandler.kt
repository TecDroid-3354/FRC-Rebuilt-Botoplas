package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.constants.Constants.isFlipped
import frc.robot.constants.FieldZones
import frc.robot.constants.RobotConstants
import net.tecdroid.util.stateMachine.Phase
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States.*
import org.littletonrobotics.junction.AutoLogOutput

class StatesHandler(private val superstructure: Superstructure, private val controller: CommandXboxController) {
    private val stateMachine: StateMachine = StateMachine(NeutralState)

    init {
        configureStatesConditions()
        configureStatesCommands()
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
            Phase.Teleop)

        stateMachine.addCondition(
            { controller.rightBumper().asBoolean },
            IntakeState,
            Phase.Teleop)

        stateMachine.addCondition(
            { superstructure.isInsideZone(FieldZones.NEUTRAL_ZONE) && controller.rightTrigger().asBoolean },
            AssistState,
            Phase.Teleop)

        stateMachine.addCondition(
            { superstructure.isInsideZone(
                if (isFlipped.invoke()) FieldZones.RED_ALLIANCE_ZONE else FieldZones.BLUE_ALLIANCE_ZONE
            ) },
            PrepShootingState,
            Phase.Teleop
        )

        stateMachine.addCondition(
            { controller.rightTrigger().asBoolean },
            ShootState,
            Phase.Teleop
        )

        stateMachine.addCondition(
            { controller.povLeft().asBoolean } ,
            EmergencyShootState,
            Phase.Teleop
        )
    }

    private fun configureStatesCommands() {
        //IntakeState.setDefaultCommand(superstructure.intakeStateCMD())

        //PrepShootingState.setDefaultCommand(superstructure.preShootingStateHoodInterpolationCMD())

        //ShootState.setInitialCommand(superstructure.driveTargetingHUB())
        //ShootState.setDefaultCommand(superstructure.shootStateSequenceDefaultCMD())
        //ShootState.setEndCommand(superstructure.driveFollowingDriverInput())
    }

    private fun configureBindings() {
        controller.start().onTrue(superstructure.resetDrivePose())

        controller.rightBumper().whileTrue(superstructure.intakeStateCMD())
            .onFalse(superstructure.disableIntake())

        controller.rightTrigger().whileTrue(
            ParallelCommandGroup(
                superstructure.noStateIndexerOnly(),
                superstructure.noStateShootOnly(),
        )).onFalse(superstructure.disableIndexer().andThen(superstructure.disableShooter()))

        controller.leftBumper().whileTrue(superstructure.driveTargetingHUB())

        controller.b().whileTrue(superstructure.noStateShootOnly())
            .onFalse(superstructure.disableShooter())

        controller.a().whileTrue(superstructure.noStateIndexerOnly())
            .onFalse(superstructure.disableIndexer())

        controller.y().onTrue(superstructure.noStateIntakeDeployableOnlyEnable())

        controller.x().onTrue(superstructure.noStateIntakeDeployableOnlyDisable())
    }
}