package frc.template.utils.subsystemUtils.generic

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.template.utils.interfaces.MeasurableSubsystem
import frc.template.utils.interfaces.VoltageControlledSubsystem
import frc.template.utils.subsystemUtils.identification.GenericSysIdRoutine

abstract class TdSubsystem(name: String): SubsystemBase(name), MeasurableSubsystem, VoltageControlledSubsystem {
    abstract val sysIdForwardRunningCondition: () -> Boolean
    abstract val sysIdBackwardRunningCondition: () -> Boolean

    fun createIdentificationRoutine() : GenericSysIdRoutine {
        return GenericSysIdRoutine(
            name = name,
            subsystem = this,
            forwardsRunningCondition = sysIdForwardRunningCondition,
            backwardsRunningCondition = sysIdBackwardRunningCondition
        )
    }
}