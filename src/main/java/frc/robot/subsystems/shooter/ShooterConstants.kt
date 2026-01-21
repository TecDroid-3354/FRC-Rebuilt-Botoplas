package frc.robot.subsystems.shooter

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.NumericId
import frc.template.utils.mechanical.Reduction
import frc.template.utils.seconds
import java.util.Optional

object ShooterConstants {
    object Identification {
        val LeadMotorID = NumericId(1)
        val MotorFollowerOneID = NumericId(2)
        val MotorFollowerTwoID = NumericId(3)
        val MotorFollowerThreeID = NumericId(4)

    }

    object PhysicalLimits {
        val Reduction: Reduction = Reduction(0.0)

    }

    object Configuration {
        val AMPSLimits: Current = Amps.of(40.0)
        val motorOrientation : InvertedValue = InvertedValue.Clockwise_Positive
        val neutralMode = NeutralModeValue.Brake
        val controlGains = ControlGains(0.11,0.0,0.0,0.0,0.25,0.12,0.01,0.0)

        val cruiseVelocity = RadiansPerSecond.of(100.0)
        val acceleration:Time = 0.1.seconds
        val jerkTime : Time = 0.2.seconds

        val motorsConfig = KrakenMotors.createTalonFXConfiguration(
            Optional.of(KrakenMotors.configureMotorOutputs(neutralMode, motorOrientation)),
            Optional.of(KrakenMotors.configureCurrentLimits(AMPSLimits, false, statorCurrentLimit = Amps.of(0.0))),
            Optional.of(KrakenMotors.configureSlot0(controlGains)),
            Optional.of(KrakenMotors.configureAngularMotionMagic(
                AngularMotionTargets(cruiseVelocity, acceleration,jerkTime),
                PhysicalLimits.Reduction
            ))
        )

    }

}