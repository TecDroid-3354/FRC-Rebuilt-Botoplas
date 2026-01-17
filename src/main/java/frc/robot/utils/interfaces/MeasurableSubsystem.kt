package frc.template.utils.interfaces

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.template.utils.subsystemUtils.generic.TdSubsystem

interface MeasurableSubsystem : VoltageControlledSubsystem {
    val motorPosition: Angle
    val motorVelocity: AngularVelocity
    val power: Double
}

interface AngularSubsystem {
    val angle: Angle
    val angularVelocity: AngularVelocity

    fun setAngle(targetAngle: Angle)
    fun setAngle(targetAngle: Angle, slot: Int)

    fun setAngleCommand(targetAngle: Angle): Command = Commands.runOnce(
        { setAngle(targetAngle) },
        if (this is TdSubsystem) this
        else throw IllegalStateException("Attempted to run an angular position command on a non-subsystem")
    )

    fun setAngleCommand(targetAngle: Angle, slot: Int): Command = Commands.runOnce(
        { setAngle(targetAngle, slot) },
        if (this is TdSubsystem) this
        else throw IllegalStateException("Attempted to run an angular position command on a non-subsystem")
    )
}

interface LinearSubsystem {
    val displacement: Distance
    val velocity: LinearVelocity

    fun setDisplacement(targetDisplacement: Distance)
    fun setDisplacementCommand(targetDisplacement: Distance): Command = Commands.runOnce(
        { setDisplacement(targetDisplacement) },
        if (this is TdSubsystem) this
        else throw IllegalStateException("Attempted to run a linear position command on a non-subsystem")
    )
}
