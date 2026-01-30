package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.intake.IntakeConstants.PostionMotorLimitAmps
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.NumericId
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.safety.MeasureLimits
import frc.template.utils.volts

class Intake : SubsystemBase() {

    // Motors
    private val positionMotor = OpTalonFX(NumericId(IntakeConstants.kPositionMotorId))
    private val rollerMotor = OpTalonFX(NumericId(IntakeConstants.kRollerMotorId))

    //Encoder
    private val encoder = CANcoder(IntakeConstants.encoder.id)



    // Limits

    private val limits = MeasureLimits(
        IntakeConstants.absoluteMinumum,
        IntakeConstants.absoluteMaximum
    )

    init {
        configurePositionMotor()
        matchRelativeToAbsolute()
    }

    // -------------------------------
    // Motor configuration (Phoenix 6)
    // -------------------------------
    private fun configurePositionMotor() {

        val config = TalonFXConfiguration()

        with(config) {

            MotorOutput = KrakenMotors.configureMotorOutputs(NeutralModeValue.Brake, IntakeConstants.PostionMotor.toInvertedValue())
            MotorOutput = KrakenMotors.configureMotorOutputs(NeutralModeValue.Brake, IntakeConstants.RollMotor.toInvertedValue())

            //Motor Limits in amps
            CurrentLimits = KrakenMotors.configureCurrentLimits(PostionMotorLimitAmps, true, IntakeConstants.RollerMotorLimitAmps)
            CurrentLimits = KrakenMotors.configureCurrentLimits(PostionMotorLimitAmps, true, PostionMotorLimitAmps)


            // -------------------------------
            // PID + Feedforward
            // -------------------------------

            Slot0 = KrakenMotors.configureSlot0(IntakeConstants.controlGains)

            MotionMagic = KrakenMotors.configureAngularMotionMagic(IntakeConstants.angularMotionTargets, IntakeConstants.kReduction)
        }

        // Apply configuration
        positionMotor.applyConfigAndClearFaults(config)

    }

    // -------------------------------
    // PRIVATE — Retractile component
    // -------------------------------
    private fun deploy() {
        setPosition(IntakeConstants.kDeployedPosition)
    }

    private fun retract() {
        setPosition(IntakeConstants.kRetractedPosition)
    }

    private fun setPosition(mechanismPosition: Angle) {

        // Clamp within physical limits
        val clampedAngle = limits.coerceIn(mechanismPosition) as Angle

        // Convert mechanism rotations → motor rotations
        val motorRotations = IntakeConstants.kReduction.unapply(clampedAngle)

        // Motion Magic control
        positionMotor.positionRequest(motorRotations)
    }

    private fun matchRelativeToAbsolute() {
        val absolutePosition =
            encoder.position.value + IntakeConstants.kAbsoluteEncoderOffset

        positionMotor.getMotorInstance().setPosition(absolutePosition)
    }

    // -------------------------------
    // PRIVATE — Rollers
    // -------------------------------
    private fun enableRollers() {
        rollerMotor.voltageRequest(IntakeConstants.kRollerVoltage)
    }

    private fun disableRollers() {
        rollerMotor.voltageRequest(0.0.volts)
    }

    // -------------------------------
    // PUBLIC
    // -------------------------------
    fun enableIntake() {
        deploy()
        enableRollers()
    }

    fun stopIntake() {
        retract()
        disableRollers()
    }
}
