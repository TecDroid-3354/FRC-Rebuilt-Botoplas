package frc.template.utils.interfaces

import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.template.utils.subsystemUtils.generic.TdSubsystem
import frc.template.utils.volts
import java.util.function.Supplier

interface VoltageControlledSubsystem {
    fun setVoltage(voltage: Voltage)
    fun setVoltageCommand(voltage: Supplier<Voltage>): Command = Commands.runOnce(
        { setVoltage(voltage.get()) },
        if (this is TdSubsystem) this
        else throw IllegalStateException("Attempted to set voltage output on a non-subsystem.")
    )

    fun stop() = setVoltage(0.0.volts)
    fun stopCommand(): Command = Commands.runOnce(
        ::stop,
        if(this is TdSubsystem) this
        else throw IllegalStateException("Attempted to stop a non-subsystem. This has no safety guarantees.")
    )
}