package frc.robot.constants

import com.pathplanner.lib.path.PathConstraints
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
import kotlin.math.pow

object RobotConstants {

    object RobotPhysics {
        val RobotMass       : Mass              = Kilograms.of(15.0)
        val RobotLength     : Distance          = 27.0.inches
        val RobotWidth      : Distance          = 27.0.inches
        val RobotMOI        : MomentOfInertia   = KilogramSquareMeters.of(
            (1/12) * (RobotMass.`in`(Kilograms)) * (RobotLength.`in`(Meters).pow(2) + RobotWidth.`in`(Meters).pow(2))
        )
        const val WHEEL_COF : Double            = 1.2
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
    }

    object Control {
        val DRIVE_ROTATION_TOLERANCE_BEFORE_SHOOTING: Angle = 5.0.degrees
    }
}