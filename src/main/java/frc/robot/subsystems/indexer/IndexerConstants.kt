package frc.robot.subsystems.indexer

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.subsystems.shooter.IntakeConstants
import frc.robot.subsystems.shooter.Shooter
import frc.robot.utils.controlProfiles.LoggedTunableNumber
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.volts
import java.util.Optional

object IndexerConstants {
        /**
         * Unique ID of every component in the shooter
         */
        object Identification {
                const val BOTTOM_ROLLERS_ID   : Int = 5
                const val LATERAL_ROLLERS_ID  : Int = 6
        }

        /**
         * Pre-defined voltage targets for each set of rollers.
         * Only these targets should be used since velocity is constant.
         */
        object VoltageTargets {
                var BottomRollerVoltage         : Voltage = 6.0.volts
                var LateralRollerVoltage        : Voltage = 6.0.volts
        }

        object Tunables {
                val bottomRollerVoltage         : LoggedTunableNumber = LoggedTunableNumber("${{Telemetry.INDEXER_TAB}}/Bottom Rollers voltage", 10.0)
                val lateralRollerVoltage        : LoggedTunableNumber = LoggedTunableNumber("${Telemetry.INDEXER_TAB}/Lateral Rollers voltage", 10.0)
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
