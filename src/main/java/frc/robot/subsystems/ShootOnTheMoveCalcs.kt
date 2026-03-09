package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Time
import frc.robot.constants.Constants.isFlipped
import frc.robot.constants.FieldConstants.HUB
import frc.robot.constants.RobotConstants.RobotMeasures
import java.util.function.Supplier


class ShootOnTheMoveCalcs (
    private val drivePose               : Supplier<Pose2d>,
    private val driveChassisSpeeds      : Supplier<ChassisSpeeds>,
    private val projectileFlightTime    : Supplier<Time>,

    ) {
    private val HUB_TARGET = if (isFlipped.invoke()) HUB.RED_HUB_VECTOR else HUB.BLUE_HUB_VECTOR

    /**
     * TODO() = Update Comment. Extracted from Mechanical Advantage 2026 GeomUtil file.
     * Transforms a velocity along a translation.
     *
     * @return The new velocity
     */
//    fun getShooterVelocity(): ChassisSpeeds {
//        return ChassisSpeeds(
//            driveChassisSpeeds.get().vxMetersPerSecond
//                    + driveChassisSpeeds.get().omegaRadiansPerSecond
//                        * (RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.getY() * drivePose.get().rotation.getCos()
//                        - RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.getX() * drivePose.get().rotation.getSin()),
//            driveChassisSpeeds.get().vyMetersPerSecond
//                    + driveChassisSpeeds.get().omegaRadiansPerSecond
//                        * (RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.getX() * drivePose.get().rotation.getCos()
//                        - RobotMeasures.TRANSFORM_ROBOT_TO_SHOOTER_FRAME.getY() * drivePose.get().rotation.getSin()),
//            driveChassisSpeeds.get().omegaRadiansPerSecond
//        )
//    }

//    fun getAdjustedTargetAngle(): Angle {
//        return Radians.of(HUB_TARGET.minus(
//            getShooterVelocity().times(projectileFlightTime.get().`in`(Seconds))
//        ).angle.radians)
//    }
}