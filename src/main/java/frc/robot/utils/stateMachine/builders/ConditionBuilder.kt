package net.tecdroid.util.stateMachine.builders

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.subsystems.intake.Intake
import net.tecdroid.util.stateMachine.Phase
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States
import java.util.UUID

class ConditionBuilder(private val stateMachine: StateMachine,
                       private var condition: () -> Boolean,
                       private val targetState: States,
                       private val executionPhase: Phase,
                       private val weight: Int
) {

    private var storedCondition: Condition? = null

    data class Condition( // To identify a specific condition and have the possibility of delete it
        var condition: () -> Boolean,
        var targetState: States,
        var weight: Int
    ) {
        val id: UUID = UUID.randomUUID()
    }

    fun storeCondition()  {
        val condition = Condition(condition, targetState, weight)

        stateMachine.getConditionList(executionPhase).add(condition)

        // store the current condition object
        storedCondition = condition
    }

    private fun removeStoredCondition(condition: Condition?) {
        condition?.let {
            stateMachine.getConditionList(executionPhase).removeIf { it.id == condition.id }
        }
    }

    fun unlessIs(state: States): ConditionBuilder {
        val oldCondition = condition

        condition = { oldCondition() && !stateMachine.isState(state)() }

        removeStoredCondition(storedCondition)
        storeCondition()

        return this
    }

    fun andIs(state: States): ConditionBuilder {
        val oldCondition = condition

        condition = { oldCondition() && stateMachine.isState(state)() }

        removeStoredCondition(storedCondition)
        storeCondition()

        return this
    }

    fun andThen(command : Command): ConditionBuilder {
        CommandScheduler.getInstance().schedule(command)
        return this
    }
}