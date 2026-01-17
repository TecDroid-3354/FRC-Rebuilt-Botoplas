package frc.robot.subsystems.exampleSubsystem

import com.ctre.phoenix6.signals.NeutralModeValue
import frc.template.utils.amps
import frc.template.utils.devices.NumericId
import frc.template.utils.devices.RotationalDirection
import frc.template.utils.mechanical.Reduction

object ExampleConstants {

        object Identification {

                val MOTOR_ID = NumericId(1)

                val CAN_BUS = "canivore"
        }

        object Configuration {

                val NEUTRAL_MODE_VALUE = NeutralModeValue.Brake

                val INVERTED_VALUE = RotationalDirection.Clockwise
        }

        object CurrentLimits {

                val MOTOR_CURRENT_LIMIT = 40.0.amps
        }

        object PhysicalLimits {

                val GEAR_RATIO = Reduction(1.0)
        }
}