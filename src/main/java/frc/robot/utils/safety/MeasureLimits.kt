package frc.template.utils.safety

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit

val pidOutputRange = (-1.0) .. (1.0)

fun <U: Unit> clamp(min: Measure<U>, max: Measure<U>, value: Measure<U>): Measure<U> {
    return if (value > max) max else if (value < min) min else value
}

data class MeasureLimits <U: Unit>(
    val minimum: Measure<U>,
    val maximum: Measure<U>,
) {
    constructor(range: ClosedRange<Measure<U>>) : this(range.start, range.endInclusive)

    init {
        require(maximum.gt(minimum)) { "Maximum cannot be less than Minimum" }
    }

    /**
     * Clamps the desired value within this object limits.
     * NOTE: you have to perform a cast at the end matching your [value] type.
     * @param value The value you want to clamp within limits.
     * @return The clamped value.
     */
    fun coerceIn(value: Measure<U>) = clamp(minimum, maximum, value)

    operator fun contains(measure: Measure<U>): Boolean {
        return minimum.lt(measure) && maximum.gt(measure)
    }

    operator fun compareTo(measure: Measure<U>): Int =
        if (measure in this) 0
        else if (measure > maximum) -1
        else 1

}

operator fun <U: Unit> Measure<U>.compareTo(measureLimits: MeasureLimits<U>): Int {
    return if (this in measureLimits) 0
    else if (this > measureLimits.maximum) return 1
    else -1
}
