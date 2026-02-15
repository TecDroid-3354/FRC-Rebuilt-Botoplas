package frc.robot.constants

import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Mass
import frc.template.utils.inches

object RobotConstants {
    object AutonomousConstants {
        val RobotMass: Mass = Kilograms.of(15.0)
        val RobotLength: Distance = 27.0.inches
        val RobotWidth: Distance = 27.0.inches
        val RobotMOI: Double = 1.652
    }
}