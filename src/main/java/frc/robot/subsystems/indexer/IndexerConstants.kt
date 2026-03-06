package frc.robot.subsystems.indexer

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.volts
import java.util.Optional

object IndexerConstants {
        /**
         * Unique ID of every component in the shooter
         */
        object Identification {
                const val INDEXER_ROLLERS_ID   : Int = 30
        }

        /**
         * Pre-defined voltage targets for each set of rollers.
         * Only these targets should be used since velocity is constant.
         */
        object VoltageTargets {
                var RollersVoltage         : Voltage = Tunables.indexerRollersVoltage.get().volts
        }

        object Tunables {
                val indexerRollersVoltage         : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Rollers voltage", 6.0)
        }

        /**
         * All MOTOR configuration. Every field must be written privately and separately to be called
         * in a public [com.ctre.phoenix6.configs.TalonFXConfiguration]
         */
        object Configuration {
                // ---------------------------------
                // PRIVATE — Motor Outputs
                // ---------------------------------
                private val neutralMode         : NeutralModeValue = NeutralModeValue.Coast
                private val motorOrientation    : InvertedValue = InvertedValue.CounterClockwise_Positive

                // ---------------------------------
                // PRIVATE — Current Limits
                // ---------------------------------
                private val supplyCurrentLimits : Current = Amps.of(40.0)
                private val statorCurrentLimits : Current = Amps.of(40.0)
                private val statorCurrentEnable : Boolean = false

                val motorConfig = KrakenMotors.createTalonFXConfiguration(
                        Optional.of(
                                KrakenMotors.configureMotorOutputs(neutralMode, motorOrientation)),
                        Optional.of(
                                KrakenMotors.configureCurrentLimits(
                                        supplyCurrentLimits,
                                        statorCurrentEnable,
                                        statorCurrentLimits
                                )),
                        Optional.empty(), // slot0 (PID not used)
                        Optional.empty()  // motionMagic (not used)
                )
        }

        /**
         * Used to store AdvantageScope's tab in which to display [Indexer] data.
         * Also used for [frc.robot.utils.controlProfiles.LoggedTunableNumber]
         */
        object Telemetry {
                const val INDEXER_TAB: String = "Indexer"
                const val INDEXER_ENABLED_FIELD: String = "${INDEXER_TAB}/Indexer Enabled"
                const val INDEXER_CONNECTED_ALERTS_FIELD: String = "${INDEXER_TAB}/Indexer Alerts"
        }
}
