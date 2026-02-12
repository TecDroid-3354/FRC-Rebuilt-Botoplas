package frc.template.utils.devices

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Current
import frc.robot.subsystems.exampleSubsystem.ExampleConstants.Identification.CAN_BUS
import frc.template.utils.Sprocket
import frc.template.utils.amps
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.controlProfiles.LinearMotionTargets
import frc.template.utils.mechanical.Reduction
import java.util.Optional

/**
 * Class meant to provide an easier, more concise way to configure [TalonFX] motor controllers within the project.
 * Please be aware of the default config written in this file, as that is the configuration that would be applied
 * for any missing user-provided configuration when creating a [TalonFX].
 */
object KrakenMotors {
    private val defaultConfig = TalonFXConfiguration()

    /**
     * Called after the primary constructor. Used to initialize the default config for all [TalonFX] motor controllers.
     */
    init {
        with(defaultConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0.amps)
                .withStatorCurrentLimitEnable(false)
                .withStatorCurrentLimit(120.0.amps)
        }
    }

    /**
     * Creates a [TalonFX] with project default, not Talon's, configuration, this means:
     * [NeutralModeValue] = Brake.
     * [InvertedValue] = CounterClockwise Positive.
     * [CurrentLimitsConfigs] = Supply current limit enabled with 40 amps, stator current limit disabled.
     * @param id The ID for the motor controller.
     * @param canBusName The canBus that the motor is in.
     * @return A [TalonFX] motor controller with the configuration described above. Sticky faults are cleared.
     */
    fun createDefaultTalon(id: Int, canBusName: String): TalonFX {
        val talon = TalonFX(id, CANBus(canBusName))
        talon.clearStickyFaults()
        talon.configurator.apply(defaultConfig)
        return talon
    }

    /**
     * Creates a custom [TalonFXConfiguration]. Note that you can type [Optional.empty] for any configuration, causing
     * it to stay with the project's default config in said configuration, if applicable.
     * @param motorOutputs The desired [MotorOutputConfigs]. Note that you can use [configureMotorOutputs].
     * @param currentLimits The desired [CurrentLimitsConfigs]. Note that you can use [configureCurrentLimits].
     * @param slot0 The desired [Slot0Configs]. Note that you can use [configureSlot0].
     * @param motionMagic The desired [MotionMagicConfigs]. Note that you can use [configureAngularMotionMagic] or [configureLinearMotionMagic]
     * @return A [TalonFXConfiguration] with the desired configurations.
     */
    fun createTalonFXConfiguration(motorOutputs: Optional<MotorOutputConfigs>, currentLimits: Optional<CurrentLimitsConfigs>,
                                   slot0: Optional<Slot0Configs>, motionMagic: Optional<MotionMagicConfigs>)
            : TalonFXConfiguration {
        val talonConfig = TalonFXConfiguration()

        if (motorOutputs.isPresent) {
            talonConfig.MotorOutput = motorOutputs.get()
        } else { talonConfig.MotorOutput = defaultConfig.MotorOutput }

        if (currentLimits.isPresent) {
            talonConfig.CurrentLimits = currentLimits.get()
        } else { talonConfig.CurrentLimits = defaultConfig.CurrentLimits }

        if (slot0.isPresent) {
            talonConfig.Slot0 = slot0.get()
        } else { talonConfig.Slot0 = defaultConfig.Slot0 }

        if (motionMagic.isPresent) {
            talonConfig.MotionMagic = motionMagic.get()
        } else { talonConfig.MotionMagic = defaultConfig.MotionMagic }

        return talonConfig
    }

    /**
     * Takes the desired motor outputs of a [TalonFXConfiguration] and returns a [MotorOutputConfigs] with said
     * configurations.
     * @param neutralModeValue The desired [NeutralModeValue], either Brake or Coast.
     * @param invertedValue The desired [InvertedValue], either CounterClockwise_Positive or Clockwise_Positive.
     * @return A [MotorOutputConfigs] with the desired [NeutralModeValue] and [InvertedValue].
     */
    fun configureMotorOutputs(neutralModeValue: NeutralModeValue, invertedValue: InvertedValue): MotorOutputConfigs {
        return MotorOutputConfigs()
            .withNeutralMode(neutralModeValue)
            .withInverted(invertedValue)
    }

    /**
     * Takes the desired current limits and returns a [CurrentLimitsConfigs] with said configurations. If stator limit
     * is not desired, type null.
     * @param supplyCurrentLimit The desired supply current limit.
     * @param statorCurrentLimit The desired stator current limit. Optional.
     * @return A [CurrentLimitsConfigs] with the desired supply and stator current limits, if applicable.
     */
    fun configureCurrentLimits(supplyCurrentLimit: Current,
                               statorCurrentLimitEnabled: Boolean, statorCurrentLimit: Current): CurrentLimitsConfigs {
        return CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyCurrentLimit)
            .withStatorCurrentLimitEnable(statorCurrentLimitEnabled)
            .withStatorCurrentLimit(statorCurrentLimit)
    }

    /**
     * Takes the desired [ControlGains] and returns a [Slot0Configs] with said configurations.
     * @param controlGains The desired [ControlGains].
     * @return A [Slot0Configs] with the applied [ControlGains].
     */
    fun configureSlot0(controlGains: ControlGains): Slot0Configs {
        return Slot0Configs()
            .withKP(controlGains.p)
            .withKI(controlGains.i)
            .withKD(controlGains.d)
            .withKS(controlGains.s)
            .withKV(controlGains.v)
            .withKA(controlGains.a)
            .withKG(controlGains.g)
    }

    /**
     * Takes the desired cruise velocity, acceleration and jerk of the subsystem, then corrects for the reduction.
     * @param angularMotionTargets [AngularMotionTargets] containing the desired configuration.
     * @param reduction The subsystem's [Reduction].
     * @return A [MotionMagicConfigs] with the applied configuration.
     */
    fun configureAngularMotionMagic(angularMotionTargets: AngularMotionTargets,
                                    reduction: Reduction): MotionMagicConfigs {
        return MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(reduction.unapply(angularMotionTargets.cruiseVelocity))
                .withMotionMagicAcceleration(reduction.unapply(angularMotionTargets.acceleration))
                .withMotionMagicJerk(reduction.unapply(angularMotionTargets.jerk))
    }

    /**
     * Takes the desired cruise velocity, acceleration and jerk of the subsystem, then transforms lineal to rotational
     * targets with the [Sprocket] and corrects for the reduction.
     * @param linearMotionTargets [LinearMotionTargets] containing the desired configuration.
     * @param reduction The subsystem's [Reduction].
     * @param sprocket The subsystem's [Sprocket] to convert from linear to rotational targets.
     * @return A [MotionMagicConfigs] with the applied configuration.
     */
    fun configureLinearMotionMagic(linearMotionTargets: LinearMotionTargets,
                                   reduction: Reduction, sprocket: Sprocket): MotionMagicConfigs {
        return MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(reduction.unapply(linearMotionTargets.angularVelocity(sprocket)))
                .withMotionMagicAcceleration(reduction.unapply(linearMotionTargets.angularAcceleration(sprocket)))
                .withMotionMagicJerk(reduction.unapply(linearMotionTargets.angularJerk(sprocket)))
    }
}
