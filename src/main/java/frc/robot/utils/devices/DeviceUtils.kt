package frc.template.utils.devices

import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import frc.template.utils.devices.RotationalDirection.Clockwise
import frc.template.utils.devices.RotationalDirection.Counterclockwise

/**
 * Allows grouping the properties of a motor
 */
data class MotorProperties(
    val positiveDirection: RotationalDirection,
    val maxAngularVelocity: AngularVelocity,
    val efficiencyCurveMax: Double
)

/**
 * Contains an assortment FRC legal motor properties
 */
object Motors {
    val neo = MotorProperties(Counterclockwise, RotationsPerSecond.of(94.6), 87.5)
    val krakenX60 = MotorProperties(Counterclockwise, RotationsPerSecond.of(100.0), 85.0)
}

data class EncoderProperties(
    val positiveDirection: RotationalDirection
)

object Encoders {
    val throughBore = EncoderProperties(Clockwise)
}
