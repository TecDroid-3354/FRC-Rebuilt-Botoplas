package frc.template.utils.subsystemUtils.identification

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.template.utils.subsystemUtils.generic.TdSubsystem

class SysIdRoutines(
    val quasistaticForward: Command,
    val quasistaticBackward: Command,
    val dynamicForward: Command,
    val dynamicBackward: Command
) {}

class GenericSysIdRoutine(val name: String,
                          val subsystem: TdSubsystem,
                          var forwardsRunningCondition : () -> Boolean = { true },
                          var backwardsRunningCondition : () -> Boolean = { true }
    ) {
    private val position: MutAngle = Radians.mutable(0.0)
    private val velocity: MutAngularVelocity = RadiansPerSecond.mutable(0.0)
    private val voltage: MutVoltage = Volts.mutable(0.0)

    private val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            subsystem::setVoltage,
            { log: SysIdRoutineLog ->
                log.motor(name)
                    .voltage(voltage.mut_replace(RobotController.getBatteryVoltage() * subsystem.power, Volts))
                    .angularPosition(position.mut_replace(subsystem.motorPosition))
                    .angularVelocity(velocity.mut_replace(subsystem.motorVelocity))
            },
            subsystem
        )
    )

    private fun createQuasistaticTest(direction: SysIdRoutine.Direction) = routine.quasistatic(direction)
    private fun createDynamicTest(direction: SysIdRoutine.Direction) = routine.dynamic(direction)

    fun createTests() = SysIdRoutines(
        quasistaticForward = createQuasistaticTest(SysIdRoutine.Direction.kForward).onlyWhile(forwardsRunningCondition),
        quasistaticBackward = createQuasistaticTest(SysIdRoutine.Direction.kReverse).onlyWhile(backwardsRunningCondition),
        dynamicForward = createDynamicTest(SysIdRoutine.Direction.kForward).onlyWhile(forwardsRunningCondition),
        dynamicBackward = createDynamicTest(SysIdRoutine.Direction.kReverse).onlyWhile(backwardsRunningCondition),
    )
}
