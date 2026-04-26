package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.Sprocket
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.controlProfiles.LinearMotionTargets
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.inches
import frc.template.utils.mechanical.Reduction
import frc.template.utils.meters
import frc.template.utils.safety.MeasureLimits
import frc.template.utils.seconds
import frc.template.utils.volts
import java.util.Optional

object IntakeConstants {
    /**
     * Unique ID of every component in the shooter
     */
    object Identification {
        const val LEAD_DEPLOY_MOTOR_ID          : Int = 23 // Right Motor
        const val LEAD_ROLLERS_MOTOR_ID         : Int = 21
        const val FOLLOWER_ROLLERS_MOTOR_ID     : Int = 22 // Left Motor
    }

    /**
     * Every physical aspect needed to be considered in code
     */
    object PhysicalLimits {
        val DeployableReduction                   : Reduction = Reduction(50.0/3.0)
        val DeployableLimits                      : MeasureLimits<DistanceUnit> = MeasureLimits(0.06.meters, 0.307.meters)

        val RollersReduction : Reduction = Reduction(7.0/3.0)
    }

    /**
     * Idle deployable positions for each intake state: retracted and deployed
     */
    object RetractileAngles {
        val ClusteredDisplacement               : Distance = 0.07.meters
        val DeployedDisplacement                : Distance = 0.295.meters // 0.303354
        val DeployableDisplacementDelta         : Distance = 0.05.meters
    }

    /**
     * Pre-defined voltage targets for the [frc.robot.subsystems.intake.Intake] rollers.
     * Only these targets should be used since velocity is constant.
     */
    object VoltageTargets {
        var EnabledRollersVoltage           : Voltage = Tunables.enabledRollersVoltage.get().volts
        var ClusteringRollersVoltage        : Voltage = Tunables.clusteringRollersVoltage.get().volts
        var IdleRollersVoltage              : Voltage = Tunables.idleRollersVoltage.get().volts
    }

    /**
     * Contains all tunable fields. These can be changed live through Elastic and displayed through AdvantageScope.
     */
    object Tunables {
        val motorkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kP", 0.5)
        val motorkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kI", 0.0)
        val motorkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kD", 0.0)
        val motorkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Motors kF", 0.0)

        val enabledRollersVoltage      : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Enabled Rollers Voltage", 10.0)
        val clusteringRollersVoltage   : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Clustering Rollers Voltage", 8.5)
        val idleRollersVoltage         : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INTAKE_TAB}/Idle Rollers Voltage", 0.0)
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
        private val neutralMode                     : NeutralModeValue = NeutralModeValue.Coast
        private val deployableMotorOrientation      : InvertedValue = InvertedValue.CounterClockwise_Positive
        private val rollerMotorOrientation          : InvertedValue = InvertedValue.Clockwise_Positive

        // ---------------------------------
        // PRIVATE — Current Limits
        // ---------------------------------
        private val supplyCurrentLimits : Current = Amps.of(25.0)
        private val statorCurrentLimits : Current = Amps.of(40.0)
        private val statorCurrentEnable : Boolean = false

        // ---------------------------------
        // PUBLIC — Slot Configs
        // ---------------------------------
        val deployControlGains                : ControlGains = ControlGains(
            p = Tunables.motorkP.get(), i = Tunables.motorkI.get(), d = Tunables.motorkD.get(), f = Tunables.motorkF.get(),
            s = 0.22427, v = 0.078651, a = 0.0069529, g = 0.063911)

        val clusterControlGains               : ControlGains = ControlGains(
            p = 0.175, i = 0.0, d = 0.0, f = 0.0,
            s = 0.22427, v = 0.078651.div(4.5), a = 0.0069529, g = 0.063911)

        // ---------------------------------
        // PRIVATE — Motion Magic
        // ---------------------------------
        private val deployCruiseVelocity      : LinearVelocity = Units.MetersPerSecond.of(1.0)
        private val deployAcceleration        : Time = 0.1.seconds
        private val deployJerktime            : Time = 0.1.seconds

        val deployableMotorSprocket   : Sprocket = Sprocket.fromRadius(3.0.inches.div(2.0))

        val deployMotionMagic         : MotionMagicConfigs =
            KrakenMotors.configureLinearMotionMagic(
                LinearMotionTargets(deployCruiseVelocity, deployAcceleration, deployJerktime),
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
            Optional.of(KrakenMotors.configureSlot0(deployControlGains)),
            Optional.of(KrakenMotors.configureSlot1(clusterControlGains)),
            Optional.of(deployMotionMagic)
        )

        val rollerMotorConfig = KrakenMotors.createTalonFXConfiguration(
            Optional.of(KrakenMotors.configureMotorOutputs(neutralMode, rollerMotorOrientation)),
            Optional.of(KrakenMotors.configureCurrentLimits(
                supplyCurrentLimits,
                statorCurrentEnable,
                statorCurrentLimits)),
            Optional.empty(),
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
        const val INTAKE_DEPLOYABLE_MOTOR_STALL_FLAG: String = "${INTAKE_TAB}/Deployable Stall Flag"
    }
}