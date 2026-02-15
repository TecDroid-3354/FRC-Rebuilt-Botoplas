package frc.robot.subsystems.shooter

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.degrees
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.ThroughBoreBrand
import frc.template.utils.mechanical.Reduction
import frc.template.utils.safety.MeasureLimits
import frc.template.utils.seconds
import frc.template.utils.volts
import java.util.Optional

object IntakeConstants {
    /**
     * Unique ID of every component in the shooter
     */
    object Identification {
        const val ROLLERS_MOTOR_ID          : Int = 1
        const val LEAD_DEPLOY_MOTOR_ID      : Int = 2
        const val FOLLOWER_DEPLOY_MOTOR_ID  : Int = 3
        const val ABSOLUTE_ENCODER_ID       : Int = 4
    }

    /**
     * Every physical aspect needed to be considered in code
     */
    object PhysicalLimits {
        val Reduction                   : Reduction = Reduction(1.0)
        val Limits                      : MeasureLimits<AngleUnit> = MeasureLimits(0.0.degrees, 90.0.degrees)
        val DeployableAngleDelta     : Angle = 15.0.degrees  // The acceptable error before enabling rollers.
    }

    /**
     * Idle deployable positions for each intake state: retracted and deployed
     */
    object RetractileAngles {
        val RetractedAngle               : Angle = 0.0.degrees
        val DeployedAngle                : Angle = 90.0.degrees
    }

    /**
     * Pre-defined voltage targets for the [frc.robot.subsystems.intake.Intake] rollers.
     * Only these targets should be used since velocity is constant.
     */
    object VoltageTargets {
        var EnabledRollersVoltage           : Voltage = Tunables.enabledRollersVoltage.get().volts
        var IdleRollersVoltage              : Voltage = Tunables.idleRollersVoltage.get().volts
    }

    /**
     * Contains all tunable fields. These can be changed live through Elastic and displayed through AdvantageScope.
     */
    object Tunables {
        val motorkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kP", 0.1)
        val motorkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kI", 0.0)
        val motorkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kD", 0.0)
        val motorkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kF", 0.0)

        val enabledRollersVoltage   : LoggedTunableNumber = LoggedTunableNumber("${{Telemetry.INTAKE_TAB}}/Enabled Rollers Voltage", 6.0)
        val idleRollersVoltage      : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Idle Rollers Voltage", 0.0)
    }

    /**
     * All MOTOR and ENCODER configuration. Every field must be written privately and separately to be called
     * in a public [com.ctre.phoenix6.configs.TalonFXConfiguration]
     */
    object Configuration {
        // ----------------------------------------
        // PUBLIC — Absolute Encoder Configurations
        // ----------------------------------------
        object AbsoluteEncoder {
            val offset                  : Angle = 0.0.degrees
            val inverted                : Boolean = false
            val brand                   : ThroughBoreBrand = ThroughBoreBrand.WCP
        }

        // ---------------------------------
        // PRIVATE — Motor Outputs
        // ---------------------------------
        private val neutralMode                     : NeutralModeValue = NeutralModeValue.Brake
        private val deployableMotorsOrientation     : InvertedValue = InvertedValue.Clockwise_Positive
        private val rollerMotorOrientation          : InvertedValue = InvertedValue.Clockwise_Positive

        // ---------------------------------
        // PRIVATE — Current Limits
        // ---------------------------------
        private val supplyCurrentLimits : Current = Amps.of(40.0)
        private val statorCurrentLimits : Current = Amps.of(40.0)
        private val statorCurrentEnable : Boolean = false

        // ---------------------------------
        // PUBLIC — Slot 0
        // ---------------------------------
        val controlGains                : ControlGains = ControlGains(
            p = Tunables.motorkP.get(), i = Tunables.motorkI.get(), d = Tunables.motorkD.get(), f = Tunables.motorkF.get(),
            s = 0.25, v = 0.12, a = 0.01, g = 0.0)

        // ---------------------------------
        // PRIVATE — Motion Magic
        // ---------------------------------
        private val cruiseVelocity      : AngularVelocity = RadiansPerSecond.of(100.0)
        private val acceleration        : Time = 0.1.seconds
        private val jerkTime            : Time = 0.2.seconds

        // -----------------------------------
        // PUBLIC — Motor Configuration Object
        // -----------------------------------
        val deployableMotorsConfig = KrakenMotors.createTalonFXConfiguration(
            Optional.of(KrakenMotors.configureMotorOutputs(neutralMode, deployableMotorsOrientation)),
            Optional.of(KrakenMotors.configureCurrentLimits(
                supplyCurrentLimits,
                statorCurrentEnable,
                statorCurrentLimits)),
            Optional.of(KrakenMotors.configureSlot0(controlGains)),
            Optional.of(KrakenMotors.configureAngularMotionMagic(
                AngularMotionTargets(cruiseVelocity, acceleration,jerkTime),
                PhysicalLimits.Reduction
            ))
        )

        val rollerMotorConfig = KrakenMotors.createTalonFXConfiguration(
            Optional.of(KrakenMotors.configureMotorOutputs(neutralMode, rollerMotorOrientation)),
            Optional.of(KrakenMotors.configureCurrentLimits(
                supplyCurrentLimits,
                statorCurrentEnable,
                statorCurrentLimits)),
            Optional.empty(),
            Optional.empty()
        )
    }

    /**
     * Used to store AdvantageScope's tab in which to display [frc.robot.subsystems.intake.Intake] data.
     * Also used for [frc.robot.utils.controlProfiles.LoggedTunableNumber]
     */
    object Telemetry {
        const val INTAKE_TAB                    : String = "Intake"
        const val INTAKE_CONNECTED_ALERTS_FIELD : String = "${INTAKE_TAB}/Intake Connection Alerts"
        const val INTAKE_VOLTAGE_FIELD          : String = "${INTAKE_TAB}/Rollers Component Voltage"
        const val INTAKE_ANGLE_FIELD            : String = "${INTAKE_TAB}/Deployable Component Angle"
        const val INTAKE_TARGET_ANGLE_FIELD     : String = "${INTAKE_TAB}/Deployable Component Target Angle"
        const val INTAKE_DEPLOYABLE_ERROR       : String = "${INTAKE_TAB}/Deployable Component Angle error"
    }
}