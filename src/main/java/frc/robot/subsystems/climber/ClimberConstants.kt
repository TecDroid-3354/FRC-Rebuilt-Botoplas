package frc.robot.subsystems.climber

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.subsystems.shooter.IntakeConstants.Telemetry.INTAKE_TAB
import frc.robot.subsystems.shooter.Shooter
import frc.template.utils.degrees
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.ThroughBoreBrand
import frc.template.utils.mechanical.Reduction
import frc.template.utils.safety.MeasureLimits
import frc.template.utils.volts
import java.util.Optional

object ClimberConstants {
        /**
         * Unique ID of every component in the shooter
         */
        object Identification {
                const val CLIMBER_MOTOR_ID   : Int = 5
                const val ABSOLUTE_ENCODER_ID   : Int = 6
        }

        /**
         * Every physical aspect needed to be considered in code
         */
        object PhysicalLimits {
                val Reduction                   : Reduction = Reduction(1.0)
                val Limits                      : MeasureLimits<AngleUnit> = MeasureLimits(0.0.degrees, 90.0.degrees)
        }

        /**
         * Idle deployable positions for each intake state: retracted and deployed
         */
        object ClimberPositions {
                val RetractedPose               : Angle = 0.0.degrees
                val ExtendedPose                : Angle = 90.0.degrees
        }

        /**
         * All MOTOR configuration. Every field must be written privately and separately to be called
         * in a public [com.ctre.phoenix6.configs.TalonFXConfiguration]
         */
        object Configuration {

                object AbsoluteEncoder {
                        val offset                  : Angle = 0.0.degrees
                        val inverted                : Boolean = false
                        val brand                   : ThroughBoreBrand = ThroughBoreBrand.WCP
                }
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
         * Used to store AdvantageScope's tab in which to display [Climber] data.
         * Also used for [frc.robot.utils.controlProfiles.LoggedTunableNumber]
         */
        object Telemetry {
                const val CLIMBER_TAB                   : String = "Climber"
                const val CLIMBER_ANGLE_FIELD           : String = "${CLIMBER_TAB}/Climber angle"
                const val CLIMBER_ERROR                 : String = "${CLIMBER_TAB}/Climber Angle error"
        }
}
