package frc.template.utils.devices

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.template.utils.rotations
import java.util.Optional

/* --------------------------------------------- */
/* --> ABSOLUTE GENERALIZATION FOR WCP & REV <-- */
/* --------------------------------------------- */
enum class ThroughBoreBrand { WCP, REV }

sealed interface ThroughBore {
    fun getAbsoluteReading() : Angle
    fun getIsConnected() : Boolean
}

/* For WCP ThroughBore 'By CANcoder'. Is literally a CANcoder */
private class WCPThroughBore(port: Int, canBusName: String) : ThroughBore {
    private val encoder = CANcoder(port, CANBus(canBusName))
    override fun getAbsoluteReading(): Angle { return encoder.absolutePosition.value }
    override fun getIsConnected(): Boolean { return encoder.isConnected }
}

/* For REV Throughbore. Is a DutyCycleEncoder */
private class REVThroughBore(port: Int) : ThroughBore {
    private val encoder = DutyCycleEncoder(port)
    override fun getAbsoluteReading(): Angle { return Rotations.of(encoder.get()) }
    override fun getIsConnected(): Boolean { return encoder.isConnected }
}

/* ---------------------- */
/* --> Actual Wrapper <-- */
/* ---------------------- */
class ThroughBoreAbsoluteEncoder(port: Int, private val offset: Angle, private val inverted: Boolean,
                                 private val brand: ThroughBoreBrand, canBusName: Optional<String>) {
    private val encoder: ThroughBore = when (brand) {
        ThroughBoreBrand.WCP -> WCPThroughBore(
            port,
            if (canBusName.isPresent) canBusName.get() else "rio"
        )
        ThroughBoreBrand.REV -> REVThroughBore(port)
    }

    private val reading : Angle
        get() = encoder.getAbsoluteReading()

    val position: Angle
        get() = (if (inverted) invertReading(reading) else reading) - offset

    val isConnected: Boolean
        get() = encoder.getIsConnected()

    private fun invertReading(angle: Angle) = (1.0.rotations - angle)
}
