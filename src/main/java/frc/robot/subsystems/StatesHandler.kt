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

//        stateMachine.addCondition(
//            { controller.rightBumper().asBoolean },
//            IntakeState,
//            Phase.Teleop)

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

//        stateMachine.addCondition(
//            { superstructure.isInsideZone(
//                if (isFlipped.invoke()) FieldZones.RED_ALLIANCE_ZONE else FieldZones.BLUE_ALLIANCE_ZONE
//            ) && controller.rightTrigger().asBoolean },
//            ShootState,
//            Phase.Teleop
//        )

        //stateMachine.addCondition(
          //  { controller.x().asBoolean },
            //GoingUnderTrench,
            //Phase.Teleop
        //)

//        stateMachine.addCondition(
//            { controller.povLeft().asBoolean } ,
//            EmergencyShootState,
//            Phase.Teleop
//        )
    }

    private fun configureStatesCommands() {
        IntakeState.setDefaultCommand(superstructure.intakeStateCMD())
        IntakeState.setEndCommand(superstructure.disableIntake())

        //PrepShootingState.setDefaultCommand(superstructure.preShootingStateHoodInterpolationCMD())

        //ShootState.setInitialCommand(superstructure.driveTargetingHUB())
        //ShootState.setDefaultCommand(superstructure.shootStateSequenceWithoutOdometryCMD())
        //ShootState.setEndCommand(superstructure.driveFollowingDriverInput())

        GoingUnderTrench.setInitialCommand(superstructure.storeHood())
//        GoingUnderTrench.setDefaultCommand(superstructure.followTrajectory(
//            superstructure.getOnTheFlyPathFromWaypoints(
//                superstructure.getTrenchOnTheFlyWaypointList(),
//                Degrees.zero()
//            )
//        ))
    }

    private fun configureBindings() {
        controller.start().onTrue(superstructure.resetDrivePose()) // Reset rotation

//        controller.rightBumper().onTrue(InstantCommand({ SignalLogger.start() }))
//        controller.leftBumper().onTrue(InstantCommand({ SignalLogger.stop() }))

        controller.povUp().onTrue(superstructure.coastSubsystems().onlyIf { DriverStation.isDisabled() }
            .ignoringDisable(true))   // Coast Intake + Hood
        controller.povDown().onTrue(superstructure.brakeSubsystems().onlyIf { DriverStation.isDisabled() }
            .ignoringDisable(true)) // Brake Intake + Hood

//        controller.a().whileTrue(superstructure.shootingStateIndexerEnableCMD())
//            .onFalse(superstructure.disableIndexer())
//        controller.b().whileTrue(superstructure.noStateShootOnly())
//            .onFalse(superstructure.disableShooter())

        controller.rightTrigger().onTrue(superstructure.shootStateSequenceWithoutOdometryCMD())
            .onFalse(superstructure.disableSubsystems())

        controller.rightBumper().onTrue(superstructure.intakeStateCMD())
            .onFalse(superstructure.disableIntake())

        controller.x().onTrue(superstructure.driveTargetingHUB())
            .onFalse(superstructure.driveFollowingDriverInput())

//        controller.y().onTrue(superstructure.noStateShootOnly())
//            .onFalse(superstructure.disableShooter())
//
//        controller.b().onTrue(superstructure.noStateIndexerOnly())
//            .onFalse(superstructure.disableIndexer())
//
//        controller.povLeft().whileTrue(superstructure.shootingStateIntakeDanceCMD())
//            .onFalse(superstructure.noStateIntakeDeployableOnlyEnable())
//
//        controller.x().onTrue(superstructure.noStateIntakeDeployableOnlyEnable())
//
//        controller.a().whileTrue(superstructure.noStateIntakeDeployableOnlyDisable())

//        controller.a().whileTrue(superstructure.hood.sysIdDynamicRoutine(SysIdRoutine.Direction.kForward))
//        controller.b().whileTrue(superstructure.hood.sysIdDynamicRoutine(SysIdRoutine.Direction.kReverse))
//        controller.y().whileTrue(superstructure.hood.sysIdQuasistaticRoutine(SysIdRoutine.Direction.kForward))
//        controller.x().whileTrue(superstructure.hood.sysIdQuasistaticRoutine(SysIdRoutine.Direction.kReverse))
    }
}