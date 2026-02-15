package frc.template.utils.interfaces

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

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
        { setAngle(targetAngle) }
    )

    fun setAngleCommand(targetAngle: Angle, slot: Int): Command = Commands.runOnce(
        { setAngle(targetAngle, slot) }
    )
}

interface LinearSubsystem {
    val displacement: Distance
    val velocity: LinearVelocity

    fun setDisplacement(targetDisplacement: Distance)
    fun setDisplacementCommand(targetDisplacement: Distance): Command = Commands.runOnce(
        { setDisplacement(targetDisplacement) }
    )
}
