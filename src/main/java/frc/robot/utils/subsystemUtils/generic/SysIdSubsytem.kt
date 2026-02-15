package frc.robot.utils.subsystemUtils.generic

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.subsystemUtils.identification.GenericSysIdRoutine

abstract class SysIdSubsystem(name: String): SubsystemBase(name) {
    // SysId condition for running forwards
    protected abstract val sysIdForwardRunningCondition: () -> Boolean
    // SysId condition for running backwards
    protected abstract val sysIdBackwardRunningCondition: () -> Boolean

    // Motor's position holder
    protected abstract val motorPosition: Angle
    // Motor's velocity holder
    protected abstract val motorVelocity: AngularVelocity
    // Motor's power holder
    protected abstract val power: Double

    protected abstract fun setVoltage(voltage: Voltage)

    protected fun createIdentificationRoutines() : GenericSysIdRoutine {
        return GenericSysIdRoutine(
            name = name,
            subsystem = this,
            forwardsRunningCondition = sysIdForwardRunningCondition,
            backwardsRunningCondition = sysIdBackwardRunningCondition,
            motorPower = { power },
            motorPosition = { motorPosition },
            motorVelocity = { motorVelocity },
            subsystemSetVoltage = { setVoltage(it) }
        )
    }
}