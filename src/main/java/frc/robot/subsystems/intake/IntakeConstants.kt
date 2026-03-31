package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.Sprocket
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.controlProfiles.LinearMotionTargets
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.inches
import frc.template.utils.mechanical.Reduction
import frc.template.utils.meters
import frc.template.utils.rotationsPerSecond
import frc.template.utils.safety.MeasureLimits
import frc.template.utils.seconds
import java.util.Optional

object IntakeConstants {
    /**
     * Unique ID of every component in the shooter
     */
    object Identification {
        const val LEAD_DEPLOY_MOTOR_ID          : Int = 22 // Right Motor
        const val LEAD_ROLLERS_MOTOR_ID         : Int = 21
        const val FOLLOWER_ROLLERS_MOTOR_ID     : Int = 23 // Left Motor
    }

    /**
     * Every physical aspect needed to be considered in code
     */
    object PhysicalLimits {
        val DeployableReduction                   : Reduction = Reduction(50.0/3.0)
        val DeployableLimits                      : MeasureLimits<DistanceUnit> = MeasureLimits(0.1.meters, 0.3.meters)

        val RollersReduction : Reduction = Reduction(7.0/3.0)
    }

    /**
     * Idle deployable positions for each intake state: retracted and deployed
     */
    object RetractileAngles {
        val ClusteredDisplacement               : Distance = 0.1.meters
        val DeployedDisplacement                : Distance = 0.25.meters
        val DeployableDisplacementDelta         : Distance = 0.05.meters
    }

    /**
     * Pre-defined voltage targets for the [frc.robot.subsystems.intake.Intake] rollers.
     * Only these targets should be used since velocity is constant.
     */
    object RPSTargets {
        var EnabledRollersRPS           : AngularVelocity = Tunables.enabledRollersRPMs.get().div(60.0).rotationsPerSecond
        var ClusteringRollersRPS        : AngularVelocity = Tunables.clusteringRollersRPMs.get().div(60.0).rotationsPerSecond
        var IdleRollersRPS              : AngularVelocity = Tunables.idleRollersRPMs.get().div(60.0).rotationsPerSecond
    }

    /**
     * Contains all tunable fields. These can be changed live through Elastic and displayed through AdvantageScope.
     */
    object Tunables {
        val motorkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kP", 0.6)
        val motorkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kI", 0.0)
        val motorkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kD", 0.0)
        val motorkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kF", 0.0)

        val enabledRollersRPMs      : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Enabled Rollers RPMs", 5_000.0)
        val clusteringRollersRPMs   : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Clustering Rollers RPMs", 1_200.0)
        val idleRollersRPMs         : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Idle Rollers RPMs", 0.0)
    }

    /**
     * All MOTOR and ENCODER configuration. Every field must be written privately and separately to be called
     * in a public [com.ctre.phoenix6.configs.TalonFXConfiguration]
     */
    object Configuration {
        // ---------------------------------
        // PUBLIC — Follower Alignment
        // ---------------------------------
        val rollerFollowerAlignment                 : MotorAlignmentValue = MotorAlignmentValue.Opposed

        // ---------------------------------
        // PRIVATE — Motor Outputs
        // ---------------------------------
        private val neutralMode                     : NeutralModeValue = NeutralModeValue.Brake
        private val deployableMotorOrientation      : InvertedValue = InvertedValue.CounterClockwise_Positive
        private val rollerMotorOrientation          : InvertedValue = InvertedValue.CounterClockwise_Positive

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
            s = 0.22427, v = 0.078651, a = 0.0069529, g = 0.063911)

        // ---------------------------------
        // PRIVATE — Motion Magic
        // ---------------------------------
        private val deployCruiseVelocity      : LinearVelocity = Units.MetersPerSecond.of(0.75)
        private val deployAcceleration        : Time = 0.1.seconds
        private val deployJerktime            : Time = 0.1.seconds

        private val clusterCruiseVelocity     : LinearVelocity = Units.MetersPerSecond.of(0.25)
        private val clusterAcceleration       : Time = 1.0.seconds
        private val clusterJerkTime           : Time = 1.0.seconds

        val deployableMotorSprocket   : Sprocket = Sprocket.fromRadius(3.25.inches.div(2.0))

        val deployMotionMagic         : MotionMagicConfigs =
            KrakenMotors.configureLinearMotionMagic(
                LinearMotionTargets(deployCruiseVelocity, deployAcceleration, deployJerktime),
                PhysicalLimits.DeployableReduction,
                deployableMotorSprocket
            )

        val clusteringMotionMagic     : MotionMagicConfigs =
            KrakenMotors.configureLinearMotionMagic(
                LinearMotionTargets(clusterCruiseVelocity, clusterAcceleration, clusterJerkTime),
                PhysicalLimits.DeployableReduction,
                deployableMotorSprocket
            )

        // -----------------------------------
        // PUBLIC — Motor Configuration Object
        // -----------------------------------
        val deployableMotorsConfig = KrakenMotors.createTalonFXConfiguration(
            Optional.of(KrakenMotors.configureMotorOutputs(neutralMode, deployableMotorOrientation)),
            Optional.of(KrakenMotors.configureCurrentLimits(
                supplyCurrentLimits,
                statorCurrentEnable,
                statorCurrentLimits)),
            Optional.of(KrakenMotors.configureSlot0(controlGains)),
            Optional.of(deployMotionMagic)
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
        const val INTAKE_TAB                        : String = "Intake"
        const val INTAKE_CONNECTED_ALERTS_FIELD     : String = "${INTAKE_TAB}/Intake Connection Alerts"
        const val INTAKE_RPM_FIELD                  : String = "${INTAKE_TAB}/Rollers Component RPMs"
        const val INTAKE_MOTOR_ANGLE_FIELD          : String = "${INTAKE_TAB}/Deployable Component Angle (Motor)"
        const val INTAKE_MOTOR_TARGET_ANGLE_FIELD   : String = "${INTAKE_TAB}/Deployable Component Target Angle (Motor)"
        const val INTAKE_DISPLACEMENT_FIELD         : String = "${INTAKE_TAB}/Deployable Component Displacement"
        const val INTAKE_TARGET_DISPLACEMENT_FIELD  : String = "${INTAKE_TAB}/Deployable Component Target Displacement"
        const val INTAKE_DEPLOYABLE_ERROR           : String = "${INTAKE_TAB}/Deployable Component Angle Error"
    }
}