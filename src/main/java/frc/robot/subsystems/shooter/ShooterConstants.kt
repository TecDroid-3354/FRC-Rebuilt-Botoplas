package frc.robot.subsystems.shooter

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.mechanical.Reduction
import frc.template.utils.rotations
import frc.template.utils.seconds
import java.util.Optional

object ShooterConstants {
    /**
     * Unique ID of every component in the shooter
     */
    object Identification {
        const val LEAD_MOTOR_LEFT_SHOOTER_FIRST_ID  : Int = 1
        const val FOLLOWER_LEFT_SHOOTER_SECOND_ID   : Int = 2
        const val FOLLOWER_LEFT_SHOOTER_THIRD_ID    : Int = 3
        const val FOLLOWER_RIGHT_SHOOTER_FIRST_ID   : Int = 4
        const val FOLLOWER_RIGHT_SHOOTER_SECOND_ID  : Int = 5
        const val FOLLOWER_RIGHT_SHOOTER_THIRD_ID   : Int = 6
    }

    /**
     * Every physical aspect needed to be considered in code
     */
    object PhysicalLimits {
        val Reduction: Reduction = Reduction(0.0)
    }

    /**
     * Contains all tunable fields. These can be changed live through Elastic and displayed through AdvantageScope.
     */
    object Tunables {
        val motorkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kP", 0.1)
        val motorkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kI", 0.0)
        val motorkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kD", 0.0)
        val motorkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kF", 0.0)
    }

    /**
     * Contains all control regarding [Shooter] velocity. This includes minimum, maximum, idle (?)
     * and interpolation points.
     */
    object Control {
        val MAX_RPS               : AngularVelocity = RotationsPerSecond.of(6_000.0 / 60)
        val MIN_RPS               : AngularVelocity = MAX_RPS.unaryMinus()

        // TODO() = Implement interpolation. Write interpolation points here.
    }

    /**
     * All MOTOR configuration. Every field must be written privately and separately to be called
     * in a public [com.ctre.phoenix6.configs.TalonFXConfiguration]
     */
    object Configuration {
        // ---------------------------------
        // PRIVATE — Motor Outputs
        // ---------------------------------
        private val neutralMode         : NeutralModeValue = NeutralModeValue.Brake
        private val motorOrientation    : InvertedValue = InvertedValue.Clockwise_Positive

        // ---------------------------------
        // PRIVATE — Current Limits
        // ---------------------------------
        private val supplyCurrentLimits : Current = Amps.of(40.0)
        private val statorCurrentLimits : Current = Amps.of(40.0)
        private val statorCurrentEnable : Boolean = false

        // ---------------------------------
        // PUBLIC — Slot 0
        // ---------------------------------
        val controlGains        : ControlGains = ControlGains(
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
        val motorsConfig = KrakenMotors.createTalonFXConfiguration(
            Optional.of(
                KrakenMotors.configureMotorOutputs(neutralMode, motorOrientation)),
            Optional.of(
                KrakenMotors.configureCurrentLimits(
                    supplyCurrentLimits,
                    statorCurrentEnable,
                    statorCurrentLimits
                )),
            Optional.of(
                KrakenMotors.configureSlot0(controlGains)),
            Optional.of(
                KrakenMotors.configureAngularMotionMagic(
                    AngularMotionTargets(cruiseVelocity, acceleration,jerkTime),
                    PhysicalLimits.Reduction
            ))
        )
    }

    /**
     * Used to store AdvantageScope's tab in which to display [Shooter] data.
     * Also used for [frc.robot.utils.controlProfiles.LoggedTunableNumber]
     */
    object Telemetry {
        const val SHOOTER_TAB: String = "Shooter"
        const val SHOOTER_RPM_FIELD: String = "${SHOOTER_TAB}/Shooter RPMs"
        const val SHOOTER_TARGET_RPM_FIELD: String = "${SHOOTER_TAB}/Shooter Target RPMs"
        const val SHOOTER_CONNECTED_ALERTS_FIELD: String = "${SHOOTER_TAB}/Shooter Connection Alerts"
    }

}