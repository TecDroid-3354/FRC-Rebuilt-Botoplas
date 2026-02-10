package frc.robot.subsystems.indexer

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Current
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.NumericId
import java.util.Optional

object IndexerConstants {
        //Kraken IDs subject to change
        object Identification {
                val BottomRollerMotorID = NumericId(5)
                val LateralRollerMotorID = NumericId(6)
        }
        //The voltage from each roller that is available in the indexer
        object Voltage {
                const val BottomRollerVoltage = 6.0
                const val LateralRollerVoltage = 6.0
        }
        //Add the configuration values of AMPS,
        //Brake or Coast
        //And the Rotational Direction
        object Configuration {
                val currentLimit: Current = Amps.of(30.0)
                val neutralMode = NeutralModeValue.Brake
                val motorOrientation = InvertedValue.Clockwise_Positive

                val motorConfig = KrakenMotors.createTalonFXConfiguration(
                        Optional.of(
                                KrakenMotors.configureMotorOutputs(
                                        neutralMode,
                                        motorOrientation
                                )
                        ),
                        Optional.of(
                                KrakenMotors.configureCurrentLimits(
                                        currentLimit,
                                        false,
                                        statorCurrentLimit = Amps.of(0.0)
                                )
                        ),
                        Optional.empty(), // slot0 (PID not used)
                        Optional.empty()  // motionMagic (not used)
                )

        }
}
