package frc.template.utils

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*

//
// Custom Units
//

data class Pixels(val count: Int) {
    companion object {
        fun of(count: Int) = Pixels(count)
    }
}

data class Percentage(val value: Double)
data class Factor(val value: Double)

//
// Extension Members
//

fun Angle.toRotation2d() = Rotation2d(this)

val Int.pixels                  : Pixels                ; get() = Pixels.of(this)
val Double.percent              : Percentage            ; get() = Percentage(this)
val Double.meters               : Distance              ; get() = Units.Meters.of(this)
val Double.inches               : Distance              ; get() = Units.Inches.of(this)
val Double.rotations            : Angle                 ; get() = Units.Rotations.of(this)
val Double.degrees              : Angle                 ; get() = Units.Degrees.of(this)
val Double.radians              : Angle                 ; get() = Units.Radians.of(this)
val Double.hertz                : Frequency             ; get() = Units.Hertz.of(this)
val Double.seconds              : Time                  ; get() = Units.Seconds.of(this)
val Double.milliseconds         : Time                  ; get() = Units.Milliseconds.of(this)
val Double.volts                : Voltage               ; get() = Units.Volts.of(this)
val Double.amps                 : Current               ; get() = Units.Amps.of(this)
val Double.degreesCelsius       : Temperature           ; get() = Units.Celsius.of(this)

val Double.metersPerSecond      : LinearVelocity        ; get() = Units.MetersPerSecond.of(this)
val Double.degreesPerSecond     : AngularVelocity       ; get() = Units.DegreesPerSecond.of(this)