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
        val motorkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kP", 0.7)
        val motorkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kI", 0.0)
        val motorkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kD", 0.0)
        val motorkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Motors kF", 0.55)

        val enabledRPMs: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Manual RPMs", 2600.0)
        val warmUpRPMs : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.SHOOTER_TAB}/Warm Up RPMs", 1500.0)
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
            1.397.meters to (2_400.0).div(60.0).rotationsPerSecond,
            2.000.meters to (2_450.0).div(60.0).rotationsPerSecond,
            2.500.meters to (2_500.0).div(60.0).rotationsPerSecond,
            3.000.meters to (2_550.0).div(60.0).rotationsPerSecond,
            3.500.meters to (2_650.0).div(60.0).rotationsPerSecond,
            4.000.meters to (2_745.0).div(60.0).rotationsPerSecond,
            4.500.meters to (2_815.0).div(60.0).rotationsPerSecond,
            5.000.meters to (2_870.0).div(60.0).rotationsPerSecond,
        )

        // Score with full much use of the Hood
        // The increment per step (0.5 meters) gets bigger as we meet the limit of Hood's range, relying on pure RPMs.
        // Pair() containing: Distance to target (meters) -> Shooter target velocity (rotations per second)}
        // Original low curvature
//        val shooterScoreLowCurvatureInterpolationPoints: Map<Distance, AngularVelocity> = mapOf<Distance, AngularVelocity>(
//            1.397.meters to (2_150.0.minus(20.0)).div(60.0).rotationsPerSecond,
//            2.000.meters to (2_300.0.minus(20.0)).div(60.0).rotationsPerSecond,
//            2.500.meters to (2_350.0.minus(20.0)).div(60.0).rotationsPerSecond,
//            3.000.meters to (2_450.0.minus(30.0)).div(60.0).rotationsPerSecond,
//            3.500.meters to (2_450.0.minus(35.0)).div(60.0).rotationsPerSecond,
//            4.000.meters to (2_475.0.minus(35.0)).div(60.0).rotationsPerSecond,
//            4.500.meters to (2_575.0.minus(40.0)).div(60.0).rotationsPerSecond,
//            5.000.meters to (2_825.0.minus(40.0)).div(60.0).rotationsPerSecond,
//        )
        val shooterScoreLowCurvatureInterpolationPoints: Map<Distance, AngularVelocity> = mapOf<Distance, AngularVelocity>(
            1.397.meters to (2_400.0.minus(45.0)).div(60.0).rotationsPerSecond,
            2.000.meters to (2_450.0.minus(45.0)).div(60.0).rotationsPerSecond,
            2.500.meters to (2_500.0.minus(45.0)).div(60.0).rotationsPerSecond,
            3.000.meters to (2_550.0.minus(45.0)).div(60.0).rotationsPerSecond,
            3.500.meters to (2_650.0.minus(45.0)).div(60.0).rotationsPerSecond,
            4.000.meters to (2_745.0.minus(45.0)).div(60.0).rotationsPerSecond,
            4.500.meters to (2_815.0.minus(45.0)).div(60.0).rotationsPerSecond,
            5.000.meters to (2_870.0.minus(45.0)).div(60.0).rotationsPerSecond,
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

        // ---------------------------------
        // PRIVATE — Current Limits
        // ---------------------------------
        private val supplyCurrentLimits : Current = Amps.of(35.0)
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
        private val cruiseVelocity      : AngularVelocity = RotationsPerSecond.of(100.0)
        private val acceleration        : Time = 0.1.seconds
        private val jerkTime            : Time = 0.1.seconds

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