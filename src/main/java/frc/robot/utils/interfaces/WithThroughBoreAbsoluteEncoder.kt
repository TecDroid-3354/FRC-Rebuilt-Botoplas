package frc.template.utils.interfaces

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.template.utils.devices.ThroughBoreAbsoluteEncoder

interface WithThroughBoreAbsoluteEncoder {
    val absoluteEncoder: ThroughBoreAbsoluteEncoder
    val absoluteAngle: Angle
        get() = absoluteEncoder.position

    fun onMatchRelativeEncodersToAbsoluteEncoders()

    fun matchRelativeEncodersToAbsoluteEncoders() {
        require(this is Subsystem) { "Classes With (a) Through Bore Absolute Encoder must be Subsystems" }
        onMatchRelativeEncodersToAbsoluteEncoders()
    }
}