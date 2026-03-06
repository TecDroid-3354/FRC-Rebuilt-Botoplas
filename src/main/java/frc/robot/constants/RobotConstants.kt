package frc.robot.constants

import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
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

    init {
        RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_MATRIX.setRow(0, Matrix(Nat.N1(), Nat.N3(),
            doubleArrayOf(cos(0.0), -sin(0.0),  0.0)))

        RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_MATRIX.setRow(1, Matrix(Nat.N1(), Nat.N3(),
            doubleArrayOf(sin(0.0), cos(0.0),  0.0))) // TODO(): Get Y offset from Robot's center to Shooter

        RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_MATRIX.setRow(2, Matrix(Nat.N1(), Nat.N3(),
            doubleArrayOf(  0.0,    0.0,    1.0)))
    }

    object Identification {
        const val ALTERNATE_CANBUS = "canivore"
    }

    object RobotPhysics {
        val RobotMass       : Mass              = Kilograms.of(15.0)
        val RobotLength     : Distance          = 27.0.inches
        val RobotWidth      : Distance          = 27.0.inches
        val RobotMOI        : MomentOfInertia   = KilogramSquareMeters.of( // As per PathPlanner rough estimate
            (1/12) * (RobotMass.`in`(Kilograms)) * (RobotLength.`in`(Meters).pow(2) + RobotWidth.`in`(Meters).pow(2))
        )
        const val WHEEL_COF : Double            = 1.2
    }

    object RobotMeasures {
        val TRANSFORM_ROBOT_TO_SHOOTER_MATRIX: Matrix<N3, N3> = Matrix(Nat.N3(), Nat.N3())
    }

    object AutonomousConfigs {
        private val maxVelocity             : LinearVelocity        = 4.0.metersPerSecond
        private val maxAcceleration         : LinearAcceleration    = 4.0.metersPerSecondPerSecond
        private val maxAngularVelocity      : AngularVelocity       = 520.0.degreesPerSecond
        private val maxAngularAcceleration  : AngularAcceleration   = 520.0.degreesPerSecondPerSecond

        val onTheFlyPathConstraints         : PathConstraints       = PathConstraints(
            maxVelocity, maxAcceleration,
            maxAngularVelocity, maxAngularAcceleration)
    }

    object AutonomousPathStrings {
        const val LEFT_TRENCH_ONE_METER_RIGHT               : String    = "1MeterPath"
        const val LEFT_TRENCH_FIVE_METERS_RIGHT_WITH_180    : String    = "5MeterWith180RotationPath"
        const val LEFT_TRENCH_AROUND_THE_WORLD              : String    = "AroundTheWorldPath"
        const val ZIG_ZAG                                   : String    = "ZigZagPath"
        const val UNDER_RIGHT_TRENCH                        : String    = "UnderRightTrench"
    }

    object DriverControllerConstants {
        const val DRIVER_CONTROLLER_PORT: Int = 0

        const val DRIVER_CONTROLLER_X_MULTIPLIER: Double = 0.8
        const val DRIVER_CONTROLLER_Y_MULTIPLIER: Double = 0.8
        const val DRIVER_CONTROLLER_Z_MULTIPLIER: Double = -(0.6)

        const val SWERVE_LOCKED_ANGLE_X_MULTIPLIER: Double = 0.5
        const val SWERVE_LOCKED_ANGLE_Y_MULTIPLIER: Double = 0.5
    }

    object Control {
        val DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING: Angle = 5.0.degrees
        val INTAKE_DEPLOYABLE_DANCE_DELTA           : Angle = 30.0.degrees
        val INTAKE_DEPLOYABLE_DANCE_TOLERANCE       : Angle = 2.0.degrees
        val SHOOTER_VELOCITY_TOLERANCE              : AngularVelocity = 1.0.rotationsPerSecond
    }

    object Telemetry {
        const val STATES_TAB = "States"
        const val STATES_CURRENT_STATE_FIELD = "${STATES_TAB}/Current State"
    }
}