package frc.template.utils.devices

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.template.utils.hertz

class OpTalonFX(id: NumericId) {

    private var canBusName = "rio"
    private val motor = KrakenMotors.createDefaultTalon(id ,canBusName)
    private var isFollower: Boolean = false

    private val voltageOutRequest = VoltageOut(0.0)
    private val motionMagicVoltage = MotionMagicVoltage(0.0)

    var position: StatusSignal<Angle> = motor.position
    var velocity: StatusSignal<AngularVelocity> =  motor.velocity
    var outputVoltage: StatusSignal<Voltage> = motor.motorVoltage
    var supplyCurrent: StatusSignal<Current> = motor.supplyCurrent
    var power: () -> Double = { motor.get() }
    var isConnected: () -> Boolean = { motor.isConnected }

    constructor(id: NumericId, canBusName: String) : this(id) {
        this.canBusName = canBusName
    }

    init {
        optimizeMotorCan()
    }

    fun voltageRequest(voltage: Voltage) {
        if (isFollower) { throw IllegalCallerException("Tried to command a follower TalonFX. Use the lead TalonFX.")}
        motor.setControl(voltageOutRequest.withOutput(voltage))
    }

    fun positionRequest(position: Angle) {
        if (isFollower) { throw IllegalCallerException("Tried to command a follower TalonFX. Use the lead TalonFX.") }
        motor.setControl(motionMagicVoltage.withPosition(position))
    }

    fun follow(leadingMotorController: TalonFX, opposeMaster: MotorAlignmentValue) {
        isFollower = true
        motor.setControl(Follower(leadingMotorController.deviceID, opposeMaster))
    }

    fun applyConfigAndClearFaults(config: TalonFXConfiguration) {
        motor.clearStickyFaults()
        motor.configurator.apply(config)
    }

    fun getMotorInstance(): TalonFX {
        return motor
    }

    private fun optimizeMotorCan() {
        with(motor) {
            position.setUpdateFrequency(100.0.hertz)
            velocity.setUpdateFrequency(100.0.hertz)
            motorVoltage.setUpdateFrequency(100.0.hertz)
            supplyCurrent.setUpdateFrequency(100.0.hertz)
            acceleration.setUpdateFrequency(50.0.hertz)
            controlMode.setUpdateFrequency(10.0.hertz)
        }
        motor.optimizeBusUtilization()
    }

    fun coast() {
        motor.setNeutralMode(NeutralModeValue.Coast)
    }

    fun brake() {
        motor.setNeutralMode(NeutralModeValue.Brake)
    }
}
