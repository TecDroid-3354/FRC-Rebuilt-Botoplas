package frc.robot.subsystems.shooter

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.mechanical.Reduction
import frc.template.utils.meters
import frc.template.utils.seconds
import java.util.Optional
import frc.template.utils.rotationsPerSecond
data class ShooterPoint(val hubDistance: Distance, val shooterRPS: AngularVelocity)

object ShooterConstants {
    /**
     * Unique ID of every component in the shooter
     */
    object Identification {
        const val LEAD_MOTOR_LEFT_SHOOTER_FIRST_ID  : Int = 40
        const val FOLLOWER_LEFT_SHOOTER_SECOND_ID   : Int = 41
        const val FOLLOWER_RIGHT_SHOOTER_FIRST_ID   : Int = 42
        const val FOLLOWER_RIGHT_SHOOTER_SECOND_ID  : Int = 43
    }

    /**
     * Every physical aspect needed to be considered in code
     */
    object PhysicalLimits {
        val Reduction: Reduction = Reduction(1.0)
    }

    /**
     * Contains all tunable fields. These can be changed live through Elastic and displayed through AdvantageScope.
     */
    object Tunables {
        // Previous control gains: 0.7, 0.0, 0.0, 0.55
        val motorNearkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kP", 0.1816) // 0.7
        val motorNearkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kI", 0.0)
        val motorNearkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kD", 0.0)
        val motorNearkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kF", 12.0) // 0.0

        val motorNearkS: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kS", 0.116) // 0.4
        val motorNearkV: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kV", 0.1193) //0.0

//        val motorFarkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kP", 0.1816) // 0.7
//        val motorFarkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kI", 0.0)
//        val motorFarkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kD", 0.0)
//        val motorFarkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kF", 12.0) // 0.0
//
//        val motorFarkS: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kS", 0.116) // 0.4
//        val motorFarkV: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/(Near) kV", 0.1193) //0.0

        val enabledRPMs: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Manual RPMs", 2400.0)
    }

    /**
     * Contains all control regarding [Shooter] velocity. This includes minimum, maximum, idle (?)
     * and interpolation points.
     */
    object Control {
        val MAX_RPS               : AngularVelocity = RotationsPerSecond.of(6_000.0 / 60)
        val MIN_RPS               : AngularVelocity = MAX_RPS.unaryMinus()

        // Score without too much use of the Hood
        // Pair() containing: Distance to target (meters) -> Shooter target velocity (rotations per second)
        val shooterScoreHighCurvatureInterpolationPoints: Map<Distance, AngularVelocity> = mapOf<Distance, AngularVelocity>(
            1.500.meters to (1_950.0).div(60.0).rotationsPerSecond,
            1.750.meters to (1_950.0).div(60.0).rotationsPerSecond,
            2.000.meters to (1_950.0).div(60.0).rotationsPerSecond,
            2.250.meters to (2_000.0).div(60.0).rotationsPerSecond,
            2.500.meters to (2_085.0).div(60.0).rotationsPerSecond,
            2.750.meters to (2_160.0).div(60.0).rotationsPerSecond,
            3.000.meters to (2_300.0).div(60.0).rotationsPerSecond,
            3.250.meters to (2_400.0).div(60.0).rotationsPerSecond,
            3.500.meters to (2_380.0).div(60.0).rotationsPerSecond,
            3.750.meters to (2_385.0).div(60.0).rotationsPerSecond,
            4.000.meters to (2_500.0).div(60.0).rotationsPerSecond,
            4.250.meters to (2_525.0).div(60.0).rotationsPerSecond,
            4.500.meters to (2_675.0).div(60.0).rotationsPerSecond,
            4.750.meters to (2_925.0).div(60.0).rotationsPerSecond,
            5.000.meters to (3_100.0).div(60.0).rotationsPerSecond,
            5.250.meters to (3_350.0).div(60.0).rotationsPerSecond,
        )

        // Assist
        // Pair() containing: Distance to target (meters) -> Shooter target velocity (rotations per second)
        val shooterAssistInterpolationPoints: Map<Distance, AngularVelocity> = mapOf<Distance, AngularVelocity>(
            1.397.meters to (2_400.0).div(60.0).rotationsPerSecond,
            2.000.meters to (2_500.0).div(60.0).rotationsPerSecond,
            2.500.meters to (2_600.0).div(60.0).rotationsPerSecond,
            3.000.meters to (2_700.0).div(60.0).rotationsPerSecond,
            3.500.meters to (2_800.0).div(60.0).rotationsPerSecond,
            4.000.meters to (2_900.0).div(60.0).rotationsPerSecond,
            4.500.meters to (3_000.0).div(60.0).rotationsPerSecond,
            5.000.meters to (3_100.0).div(60.0).rotationsPerSecond,
        )
    }

    /**
     * All MOTOR configuration. Every field must be written privately and separately to be called
     * in a public [com.ctre.phoenix6.configs.TalonFXConfiguration]
     */
    object Configuration {
        // ---------------------------------
        // PUBLIC — Follower Alignment
        // ---------------------------------
        val rightFollowerAlignment      : MotorAlignmentValue = MotorAlignmentValue.Opposed
        val leftFollowerAlignment       : MotorAlignmentValue = MotorAlignmentValue.Aligned

        // ---------------------------------
        // PRIVATE — Motor Outputs
        // ---------------------------------
        private val neutralMode         : NeutralModeValue = NeutralModeValue.Coast
        private val motorOrientation    : InvertedValue = InvertedValue.Clockwise_Positive
        private val peakDutyCycle       : Double = 0.75

        // ---------------------------------
        // PRIVATE — Current Limits
        // ---------------------------------
        private val supplyCurrentLimits : Current = Amps.of(35.0)
        private val statorCurrentLimits : Current = Amps.of(60.0)
        private val statorCurrentEnable : Boolean = true

        // ---------------------------------
        // PUBLIC — Slot 0
        // ---------------------------------
        val controlGains                : ControlGains = ControlGains(
            p = Tunables.motorNearkP.get(), i = Tunables.motorNearkI.get(), d = Tunables.motorNearkD.get(), f = Tunables.motorNearkF.get(),
            s = Tunables.motorNearkS.get(), v = Tunables.motorNearkV.get(), a = 0.026527, g = 0.0) // 0.25, 0.12, 0.1 -- 0.375, 0.1175, 0.05

        // ---------------------------------
        // PRIVATE — Motion Magic
        // ---------------------------------
        private val cruiseVelocity      : AngularVelocity = RotationsPerSecond.of(100.0)
        private val acceleration        : Time = 0.1.seconds
        private val jerkTime            : Time = 0.0.seconds

        // -----------------------------------
        // PUBLIC — Motor Configuration Object
        // -----------------------------------
        val motorsConfig = KrakenMotors.createTalonFXConfiguration(
            Optional.of(
                KrakenMotors.configureMotorOutputs(neutralMode, motorOrientation, peakDutyCycle)),
            Optional.of(
                KrakenMotors.configureCurrentLimits(
                    supplyCurrentLimits,
                    statorCurrentEnable,
                    statorCurrentLimits
                )),
            Optional.of(
                KrakenMotors.configureSlot0(controlGains)),
            Optional.empty(),
            Optional.of(
                KrakenMotors.configureAngularMotionMagic(
                    AngularMotionTargets(cruiseVelocity, acceleration, jerkTime),
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