package frc.template.utils.devices

data class NumericId(val id: Int) {

    init {
        require(id >= 0) { "Numeric ids must be positive ($id < 0)" }
    }

    operator fun compareTo(other: NumericId): Int = id.compareTo(other.id)
    operator fun plus(numericId: NumericId) : NumericId = NumericId(id + numericId.id)
    operator fun minus(numericId: NumericId): NumericId = NumericId(id - numericId.id)
    operator fun times(value: Int)          : NumericId = NumericId(id * value)

}

typealias CanId        = NumericId
typealias DigitalPort  = NumericId
typealias AnalogPort   = NumericId
typealias PwmPort      = NumericId
typealias RelayPort    = NumericId
typealias SymbolicId   = NumericId
typealias PeripheralId = NumericId