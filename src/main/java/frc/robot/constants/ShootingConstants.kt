package frc.robot.constants

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import frc.template.utils.degrees
import frc.template.utils.meters
import frc.template.utils.rotationsPerSecond
import frc.template.utils.seconds

object ShootingConstants {
    object Interpolation {
        // Taken from 6328' time of flight interpolation
        val timeOfFlightPoints: Map<Distance, Time> = mapOf<Distance, Time>(
            1.38.meters to 0.90.seconds,
            1.88.meters to 1.09.seconds,
            3.15.meters to 1.11.seconds,
            4.55.meters to 1.12.seconds,
            5.68.meters to 1.16.seconds,
        )
    }

    object DefaultParameters {
        val underTheTrenchShooterRPS: AngularVelocity = 0.0.rotationsPerSecond
        val underTheTrenchHoodAngle: Angle = 0.0.degrees
        val underTheTrenchDistanceToHUB: Distance = 0.0.meters
    }

    object Telemetry {
        private val SHOOTING_TAB                    : String = "Shooting Calcs"
        val SHOOTING_REQUESTED_DRIVE_ANGLE          : String = "${SHOOTING_TAB}/Requested Drive Angle"
        val SHOOTING_REQUESTED_HOOD_ANGLE           : String = "${SHOOTING_TAB}/Requested Hood Angle"
        val SHOOTING_REQUESTED_SHOOTER_RPS          : String = "${SHOOTING_TAB}/Requested Shooter RPS"
    }
}