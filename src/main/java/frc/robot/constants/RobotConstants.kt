package frc.robot.constants

import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N3
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
import frc.template.utils.degrees
import frc.template.utils.degreesPerSecond
import frc.template.utils.degreesPerSecondPerSecond
import frc.template.utils.inches
import frc.template.utils.metersPerSecond
import frc.template.utils.metersPerSecondPerSecond
import frc.template.utils.rotationsPerSecond
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

object RobotConstants {
    object Identification {
        const val ALTERNATE_CANBUS = "canivore"
    }

    object RobotPhysics {
        val RobotMass       : Mass              = Kilograms.of(50.0)
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

    object AutonomousPathStrings {
        // Test paths //
        const val LEFT_TRENCH_ONE_METER_RIGHT               : String    = "1MeterPath"
        const val LEFT_TRENCH_FIVE_METERS_RIGHT_WITH_180    : String    = "5MeterWith180RotationPath"
        const val LEFT_TRENCH_AROUND_THE_WORLD              : String    = "AroundTheWorldPath"
        const val ZIG_ZAG                                   : String    = "ZigZagPath"
        const val UNDER_RIGHT_TRENCH                        : String    = "UnderRightTrench"

        // Right Two Cycles //
        const val R2C_TRENCH_NEUTRAL_ZONE_1                 : String    = "TrenchToNeutralZoneCycle1Right"
        const val R2C_INTAKE_1                              : String    = "IntakeCycle1Right"
        const val R2C_SHOOT_1                               : String    = "prueba"
        const val R2C_TRENCH_NEUTRAL_ZONE_2                 : String    = "TrenchToNeutralZoneCycle2Right"
        const val R2C_INTAKE_2                              : String    = "IntakeCycle2Right"
        const val R2C_SHOOT_2                               : String    = "ShootCycle2Right"

        // Left Two Cycles //
        const val L2C_TRENCH_NEUTRAL_ZONE_1                 : String    = "TrenchToNeutralZoneCycle1Left"
        const val L2C_INTAKE_1                              : String    = "IntakeCycle1Left"
        const val L2C_SHOOT_1                               : String    = "ShootCycle1Left"
        const val L2C_TRENCH_NEUTRAL_ZONE_2                 : String    = "TrenchToNeutralZoneCycle2Left"
        const val L2C_INTAKE_2                              : String    = "IntakeCycle2Left"
        const val L2C_SHOOT_2                               : String    = "ShootCycle2Left"
    }

    object DriverControllerConstants {
        const val DRIVER_CONTROLLER_PORT: Int = 0

        const val DRIVER_CONTROLLER_X_MULTIPLIER: Double = 0.85
        const val DRIVER_CONTROLLER_Y_MULTIPLIER: Double = 0.85
        const val DRIVER_CONTROLLER_Z_MULTIPLIER: Double = -(0.6)

        const val SWERVE_LOCKED_ANGLE_X_MULTIPLIER: Double = 0.5
        const val SWERVE_LOCKED_ANGLE_Y_MULTIPLIER: Double = 0.5
    }

    object Control {
        val DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING: Angle = 2.0.degrees
        val INTAKE_DEPLOYABLE_DANCE_DELTA           : Angle = 30.0.degrees
        val INTAKE_DEPLOYABLE_DANCE_TOLERANCE       : Angle = 2.0.degrees
        val SHOOTER_VELOCITY_TOLERANCE              : AngularVelocity = 1.0.rotationsPerSecond
    }

    object Telemetry {
        const val STATES_TAB = "States"
        const val STATES_CURRENT_STATE_FIELD = "${STATES_TAB}/Current State"
    }
}