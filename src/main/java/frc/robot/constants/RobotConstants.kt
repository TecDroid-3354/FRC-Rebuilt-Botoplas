package frc.robot.constants

import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Time
import frc.template.utils.degrees
import frc.template.utils.degreesPerSecond
import frc.template.utils.degreesPerSecondPerSecond
import frc.template.utils.inches
import frc.template.utils.metersPerSecond
import frc.template.utils.metersPerSecondPerSecond
import frc.template.utils.rotationsPerSecond
import frc.template.utils.seconds
import kotlin.math.pow

object RobotConstants {
    object Identification {
        const val ALTERNATE_CANBUS = "canivore"
    }

    object RobotPhysics {
        val RobotMass       : Mass              = Kilograms.of(50.8)
        val RobotLength     : Distance          = 26.5.inches
        val RobotWidth      : Distance          = 26.5.inches
        val RobotMOI        : MomentOfInertia   = KilogramSquareMeters.of( // As per PathPlanner rough estimate
            (1/12) * (RobotMass.`in`(Kilograms)) * (RobotLength.`in`(Meters).pow(2) + RobotWidth.`in`(Meters).pow(2))
        )
        const val WHEEL_COF : Double            = 1.2
    }

    object RobotMeasures {
        // The following constant holds the shooter translational and orientation offset to the robot.
        // The x component represents the shooter offset in the x-axis relative to the robot's center.
        // The y component represents the shooter offset in the y-axis relative to the robot's center.
        // The Rotation2d θ is the orientation offset of the shooter relative to the robot's front
        // The following values construct a 3x3 matrix of the following form:
        // T(θ) = | cosθ, -sinθ, x |
        //        | sinθ, cosθ , y |
        //        | 0   , 0    , 1 |
        // Matrix is filled up with 1's and 0's to ensure homogenous coordinates are used within the system.
        val TRANSFORM_ROBOT_TO_SHOOTER_FRAME: Transform2d = Transform2d(0.0, 0.0, Rotation2d.kZero)
    }

    object AutonomousConfigs {
        private val maxVelocity             : LinearVelocity        = 2.0.metersPerSecond
        private val maxAcceleration         : LinearAcceleration    = 2.0.metersPerSecondPerSecond
        private val maxAngularVelocity      : AngularVelocity       = 320.0.degreesPerSecond
        private val maxAngularAcceleration  : AngularAcceleration   = 320.0.degreesPerSecondPerSecond

        val onTheFlyPathConstraints         : PathConstraints       = PathConstraints(
            maxVelocity, maxAcceleration,
            maxAngularVelocity, maxAngularAcceleration)
    }

    object Autonomous {
        object NameStrings {
            const val RIGHT_AUTO                    : String    = "RightAutoTwoCycles"
            const val RIGHT_MULTI_PATH_AUTO         : String    = "MultiPathRight"
            const val LEFT_AUTO                     : String    = "LeftAutoTwoCycles"
        }

        object EventTriggerStrings {
            const val INTAKE_DEPLOY                 : String    = "Intake Deploy"
            const val DISABLE_INTAKE_ROLLERS        : String    = "Disable Intake Rollers"
            const val SCORE                         : String    = "Score"
            const val ENABLE_SHOOTER                : String    = "Enable Shooter"
            const val INTAKE_LEDS                   : String    = "Intake LEDs"
            const val SCORE_LEDS                    : String    = "Score LEDs"
        }

        object ShootingConstants {
            val RIGHT_SIDE_SHOOTER_RPS              : AngularVelocity   = 2_050.0.div(60.0).rotationsPerSecond
            val RIGHT_SIDE_HOOD_ANGLE               : Angle             = 17.5.degrees
        }
    }

    object DriverControllerConstants {
        const val DRIVER_CONTROLLER_PORT: Int = 0

        const val DRIVER_CONTROLLER_X_MULTIPLIER: Double = 0.95
        const val DRIVER_CONTROLLER_Y_MULTIPLIER: Double = 0.95
        const val DRIVER_CONTROLLER_Z_MULTIPLIER: Double = -(0.6)

        const val SWERVE_LOCKED_ANGLE_X_MULTIPLIER: Double = 0.5
        const val SWERVE_LOCKED_ANGLE_Y_MULTIPLIER: Double = 0.5
    }

    object LedControllerConstants {
        const val LED_CONTROLLER_PORT: Int = 0
        const val LED_CONTROLLER_TAB: String = "Rev Blinking"
        const val LED_CONTROLLER_CONNECTED_ALERTS_FIELD: String = "${LED_CONTROLLER_TAB}/Rev Blinking Alerts"
        const val TARGET_PWM_FIELD: String = "${LED_CONTROLLER_TAB}/Target PWM"
    }

    object Control {
        val DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING: Angle             = 3.0.degrees
        val SHOOTER_VELOCITY_TOLERANCE              : AngularVelocity   = 0.8.rotationsPerSecond
        val TIME_DELTA_BEFORE_CLUSTERING            : Time              = 0.0.seconds
        val TIME_DELTA_BEFORE_CLUSTER_END           : Time              = 0.3.seconds
    }

    object LoopInfo {
        val loopPeriod: Time = 0.02.seconds
    }

    object AllianceShiftsInfo {
        private val TransitionShiftEnd: ClosedRange<Time> = Units.Seconds.of(133.0)..Units.Seconds.of(130.0)
        private val FirstShift: ClosedRange<Time> = Units.Seconds.of(108.0)..Units.Seconds.of(105.0)
        private val SecondShift: ClosedRange<Time> = Units.Seconds.of(83.0)..Units.Seconds.of(80.0)
        private val ThirdShift: ClosedRange<Time> = Units.Seconds.of(58.0)..Units.Seconds.of(55.0)
        private val LastShift: ClosedRange<Time> = Units.Seconds.of(33.0)..Units.Seconds.of(30.0)
        val Shifts: List<ClosedRange<Time>> = listOf(
            TransitionShiftEnd,
            FirstShift,
            SecondShift,
            ThirdShift,
            LastShift
        )
    }

    object Telemetry {
        const val STATES_TAB = "States"
        const val STATES_CURRENT_STATE_FIELD = "${STATES_TAB}/Current State"
    }
}