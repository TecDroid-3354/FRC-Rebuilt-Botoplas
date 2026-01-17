package frc.template.utils.mechanical

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

open class Reduction(private val ratio: Double) {

    fun apply(value: Double): Double = value / ratio
    fun unapply(value: Double): Double = value * ratio

    @Suppress("unchecked_cast")
    fun <M : Measure<out Unit>> apply(measure: M): M = measure.div(ratio) as M

    @Suppress("unchecked_cast")
    fun <M : Measure<out Unit>> unapply(measure: M): M = measure.times(ratio) as M
}
