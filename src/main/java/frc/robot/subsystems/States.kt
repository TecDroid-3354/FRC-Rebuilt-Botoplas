package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.RobotContainer
import frc.robot.constants.Constants.isFlipped
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.drivetrain.Drive
import frc.robot.subsystems.FieldZones.*
import net.tecdroid.util.stateMachine.Phase
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States.*

private enum class FieldZones {
    BLUE_ALLIANCE_ZONE,
    NEUTRAL_ZONE,
    RED_ALLIANCE_ZONE
}

class States(val drive: Drive, val controller: CommandXboxController) {

    val stateMachine: StateMachine = StateMachine(NeutralState)

    init {
        configureStatesConditions()
    }

    private fun configureStatesConditions() {
        stateMachine.addCondition(
            { isInsideZone(NEUTRAL_ZONE) },
            NeutralState,
            Phase.Teleop)

        stateMachine.addCondition(
            { isInsideZone(NEUTRAL_ZONE) && controller.rightBumper().asBoolean },
            IntakeState,
            Phase.Teleop)

        stateMachine.addCondition(
            { isInsideZone(NEUTRAL_ZONE) && controller.rightTrigger().asBoolean },
            AssistState,
            Phase.Teleop)

        stateMachine.addCondition(
            { isInsideZone(
                if (isFlipped.invoke()) RED_ALLIANCE_ZONE else BLUE_ALLIANCE_ZONE
            ) },
            PrepShootingState,
            Phase.Teleop
        )

        stateMachine.addCondition(
            { isInsideZone(
                if (isFlipped.invoke()) RED_ALLIANCE_ZONE else BLUE_ALLIANCE_ZONE
            ) && controller.rightTrigger().asBoolean },
            ShootState,
            Phase.Teleop
        )

        stateMachine.addCondition(
            { isInsideZone(
                if (isFlipped.invoke()) RED_ALLIANCE_ZONE else BLUE_ALLIANCE_ZONE
            ) && controller.b().asBoolean } ,
            EmergencyShootState,
            Phase.Teleop
        )
    }

    private fun isInsideZone(zone: FieldZones): Boolean = zone == getRobotCurrentZone()
    private fun getRobotCurrentZone(): FieldZones {
        val robotX = drive.pose.measureX

        return when {
            // If the robot's X coordinate is less than the blue trench x coordinate, then the robot is
            // inside the Blue Alliance Zone.
            robotX < FieldConstants.Trench.BLUE_TRENCH_CENTER_X -> BLUE_ALLIANCE_ZONE
            // If robot x coordinate is less than the red trench x coordinate, then the robot is in the Neutral
            // Zone as the following condition is indirectly evaluated:
            // Blue trench center X < robotX < red trench center X
            robotX < FieldConstants.Trench.RED_TRENCH_CENTER_X -> NEUTRAL_ZONE
            // If not in Blue Alliance or Neutral Zone, then robot's inside the Red Alliance Zone.
            else -> RED_ALLIANCE_ZONE
        }
    }
}