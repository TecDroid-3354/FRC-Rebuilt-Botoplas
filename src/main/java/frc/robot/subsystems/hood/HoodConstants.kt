package frc.robot.subsystems.hood

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import frc.robot.subsystems.shooter.IntakeConstants
import frc.robot.subsystems.shooter.Shooter
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.degrees
import frc.template.utils.degreesPerSecond
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.mechanical.Reduction
import frc.template.utils.meters
import frc.template.utils.safety.MeasureLimits
import frc.template.utils.seconds
import java.util.Optional

object HoodConstants {
    /**
     * Unique ID of every component in the hood
     */
    object Identification {
        const val HOOD_MOTOR_ID : Int = 51
    }

    /**
     * Every physical aspect needed to be considered in code
     */
    object PhysicalLimits {
        val Reduction           : Reduction = Reduction(9.0)
        val Limits              : MeasureLimits<AngleUnit> = MeasureLimits(0.0.degrees, 90.0.degrees)
    }

    /**
     * Contains all tunable fields. These can be changed live through Elastic and displayed through AdvantageScope.
     */
    object Tunables {
        val motorkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.HOOD_TAB}/Motors kP", 0.1)
        val motorkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.HOOD_TAB}/Motors kI", 0.0)
        val motorkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.HOOD_TAB}/Motors kD", 0.0)
        val motorkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.HOOD_TAB}/Motors kF", 0.0)
    }

    /**
     * Contains all control interpolation points for the [Hood], both distance and velocity driven.
     */
    object Control {
        // Pair() containing: Distance to target (meters) -> Hood target angle (degrees)
        val hoodDistanceDrivenInterpolationPoints: Map<Distance, Angle> = mapOf<Distance, Angle>(
            0.0.meters to 0.0.degrees,
            0.0.meters to 0.0.degrees,
            0.0.meters to 0.0.degrees,
            0.0.meters to 0.0.degrees,
            0.0.meters to 0.0.degrees,
        )

        // Pair() containing: Shooter velocity (degreesPerSecond) -> Hood target angle (degrees)
        val hoodVelocityDrivenInterpolationPoints: Map<AngularVelocity, Angle> = mapOf<AngularVelocity, Angle>(
            0.0.degreesPerSecond to 0.0.degrees,
            0.0.degreesPerSecond to 0.0.degrees,
            0.0.degreesPerSecond to 0.0.degrees,
            0.0.degreesPerSecond to 0.0.degrees,
            0.0.degreesPerSecond to 0.0.degrees,
        )
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
        val motorConfig = KrakenMotors.createTalonFXConfiguration(
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
                    AngularMotionTargets(cruiseVelocity, acceleration, jerkTime),
                    PhysicalLimits.Reduction
                ))
        )
    }

    /**
     * Used to store AdvantageScope's tab in which to display [Hood] data.
     * Also used for [frc.robot.utils.controlProfiles.LoggedTunableNumber]
     */
    object Telemetry {
        const val HOOD_TAB          : String = "Hood"
        const val HOOD_ANGLE_FIELD  : String = "${HOOD_TAB}/Hood Angle"
        const val HOOD_TARGET_ANGLE_FIELD  : String = "${HOOD_TAB}/Hood Target Angle"
        const val HOOD_CONNECTED_ALERTS_FIELD  : String = "${HOOD_TAB}/Hood Connection alert"
    }

}
