package frc.template.utils.devices

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.template.utils.hertz
import frc.template.utils.mechanical.Reduction
import frc.template.utils.safety.MeasureLimits

class OpTalonFX(private val id: Int) {

    private var canBusName = "rio"
    private val motor = KrakenMotors.createDefaultTalon(id ,canBusName)
    private var isFollower: Boolean = false

    private val voltageOutRequest = VoltageOut(0.0)
    private val motionMagicVoltageRequest = MotionMagicVoltage(0.0)
    private val motionMagicVelocityRequest = MotionMagicVelocityVoltage(0.0)

    var position: StatusSignal<Angle> = motor.position
    var velocity: StatusSignal<AngularVelocity> =  motor.velocity
    var outputVoltage: StatusSignal<Voltage> = motor.motorVoltage
    var supplyCurrent: StatusSignal<Current> = motor.supplyCurrent
    var power: () -> Double = { motor.get() }
    var isConnected: () -> Boolean = { motor.isConnected }

    constructor(id: Int, canBusName: String) : this(id) {
        this.canBusName = canBusName
    }

    init {
        optimizeMotorCan()
    }

    fun voltageRequest(voltage: Voltage) {
        if (isFollower) { throw IllegalCallerException("Tried to command a follower TalonFX [$id]. Use the lead TalonFX.")}
        motor.setControl(voltageOutRequest.withOutput(voltage))
    }

    fun velocityRequest(velocity: AngularVelocity) {
        if (isFollower) { throw IllegalCallerException("Tried to command a follower TalonFX [$id]. Use the lead TalonFX.")}
        motor.setControl(motionMagicVelocityRequest.withVelocity(velocity))
    }

    fun positionRequest(position: Angle) {
        if (isFollower) { throw IllegalCallerException("Tried to command a follower TalonFX [$id]. Use the lead TalonFX.") }
        motor.setControl(motionMagicVoltageRequest.withPosition(position))
    }

    fun positionRequestSubsystem(position: Angle, limits: MeasureLimits<AngleUnit>, reduction: Reduction) {
        if (isFollower) { throw IllegalCallerException("Tried to command a follower TalonFX [$id]. Use the lead TalonFX.") }
        val subsystemClampedAngle: Angle = limits.coerceIn(position) as Angle
        val motorTransformedAngle: Angle = reduction.unapply(subsystemClampedAngle)
        positionRequest(motorTransformedAngle)
    }

    fun stopMotor() {
        if (isFollower) { throw IllegalCallerException("Tried to command a follower TalonFX [$id]. Use the lead TalonFX.") }
        motor.stopMotor()
    }

    fun follow(leadingMotorController: TalonFX, opposeMaster: MotorAlignmentValue) {
        isFollower = true
        motor.setControl(Follower(leadingMotorController.deviceID, opposeMaster))
    }

    fun applyConfigAndClearFaults(config: TalonFXConfiguration) {
        motor.clearStickyFaults()
        motor.configurator.apply(config)
    }

    fun getMotorId(): Int {
        return motor.deviceID
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
