package frc.robot.subsystems.climber

import frc.template.utils.degrees
import frc.template.utils.rotations
import frc.template.utils.safety.MeasureLimits

object ClimberConstants {
        object Identification {
                const val LEADER_MOTOR_ID = 0
                const val FOLLOWER_MOTOR_ID = 0
                const val ABSOLUTE_ID = 0

        }

        object PhysicalLimits {
                //Gear ratio (known later)
                const val GEAR_RATIO = 7.0.rotations / 3.0.rotations

                //Positions
                const val IDLE_POSITION = 0.0.rotations
                const val CLIMB_POSITION = 1.2.rotations

                val LIMITS = MeasureLimits(0.0.rotations, 1.3.rotations)
        }

        object Configuration {
                const val CURRENT_LIMIT_AMPS = 50.0
        }

}