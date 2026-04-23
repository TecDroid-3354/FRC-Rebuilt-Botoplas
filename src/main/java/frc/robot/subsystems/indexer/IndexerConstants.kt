package frc.robot.subsystems.indexer

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
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
import frc.template.utils.rotationsPerSecond
import frc.template.utils.seconds
import java.util.Optional

object IndexerConstants {
        /**
         * Unique ID of every component in the shooter
         */
        object Identification {
                const val HOPPER_ROLLERS_ID             : Int = 30
                const val LEAD_TOWER_ROLLERS_ID         : Int = 44
                const val FOLLOWER_TOWER_ROLLERS_ID     : Int = 45
        }

        /**
         * Every physical aspect needed to be considered in code
         */
        object PhysicalLimits {
                val HopperReduction: Reduction = Reduction(20.0/3.0)
                val TowerReduction: Reduction = Reduction(1.0)
        }

        /**
         * Pre-defined velocity targets for each set of rollers.
         * Only these targets should be used since velocity is constant.
         */
        object RPSTargets {
                var HopperRollersVelocity        : AngularVelocity = Tunables.HopperRollersVelocity.get().rotationsPerSecond
                var HopperRollersIdleVelocity        : AngularVelocity = Tunables.HopperRollersIdleVelocity.get().rotationsPerSecond
                var TowerRollersVelocity         : AngularVelocity = Tunables.towerRollersVelocity.get().rotationsPerSecond
                var TowerRollersIdleVelocity         : AngularVelocity = Tunables.towerRollersIdleVelocity.get().rotationsPerSecond
        }

        object Tunables {
                val motorkP: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Motors kP", 0.5)
                val motorkI: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Motors kI", 0.0)
                val motorkD: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Motors kD", 0.0)
                val motorkF: LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Motors kF", 0.0)

                val HopperRollersVelocity         : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Hopper Rollers RPS", 45.0)
                val HopperRollersIdleVelocity         : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Hopper Rollers Idle RPS", 0.0)
                val towerRollersVelocity          : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Tower Rollers RPS", 75.0)
                val towerRollersIdleVelocity          : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Tower Rollers Idle RPS", 0.0)
        }

        /**
         * All MOTOR configuration. Every field must be written privately and separately to be called
         * in a public [com.ctre.phoenix6.configs.TalonFXConfiguration]
         */
        object Configuration {
                // ---------------------------------
                // PRIVATE — Motor Alignment values
                // ---------------------------------
                val followerTowerMotorAlignmentValue    : MotorAlignmentValue = MotorAlignmentValue.Aligned

                // ---------------------------------
                // PRIVATE — Motor Outputs
                // ---------------------------------
                private val neutralMode                 : NeutralModeValue = NeutralModeValue.Coast
                private val hopperMotorOrientation      : InvertedValue = InvertedValue.CounterClockwise_Positive
                private val towerMotorOrientation       : InvertedValue = InvertedValue.Clockwise_Positive

                // ---------------------------------
                // PRIVATE — Current Limits
                // ---------------------------------
                private val supplyCurrentLimits : Current = Amps.of(20.0)
                private val statorCurrentLimits : Current = Amps.of(20.0)
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
                private val jerkTime            : Time = 0.2.seconds

                val hopperRollersConfig = KrakenMotors.createTalonFXConfiguration(
                        Optional.of(
                                KrakenMotors.configureMotorOutputs(neutralMode, hopperMotorOrientation)),
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
                                        PhysicalLimits.HopperReduction
                                ))
                )

                val towerRollersConfig =
                        hopperRollersConfig
                                .withMotorOutput(KrakenMotors.configureMotorOutputs(neutralMode, towerMotorOrientation))
                                .withMotionMagic(KrakenMotors.configureAngularMotionMagic(
                                        AngularMotionTargets(cruiseVelocity, acceleration, jerkTime),
                                        PhysicalLimits.TowerReduction
                                ))

        }

        /**
         * Used to store AdvantageScope's tab in which to display [Indexer] data.
         * Also used for [frc.robot.utils.controlProfiles.LoggedTunableNumber]
         */
        object Telemetry {
                const val INDEXER_TAB: String = "Indexer"
                const val HOPPER_ENABLED_FIELD: String = "${INDEXER_TAB}/Hopper Enabled"
                const val TOWER_ENABLED_FIELD: String = "${INDEXER_TAB}/Tower Enabled"
                const val INDEXER_CONNECTED_ALERTS_FIELD: String = "${INDEXER_TAB}/Indexer Alerts"
        }
}
