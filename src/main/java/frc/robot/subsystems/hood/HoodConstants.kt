package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Current
import frc.template.utils.amps
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.degrees
import frc.template.utils.degreesPerSecond
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.mechanical.Reduction
import frc.template.utils.safety.MeasureLimits
import frc.template.utils.seconds
import java.util.Optional

object HoodConstants {
        object Identification {
                const val EXAMPLE_ID = 1
        }

    object PhysicalLimits {
            val REDUCTION = Reduction(1.0)
            val limits = MeasureLimits(0.0.degrees,90.0.degrees)
        }


    //valores puestos en configuration están en aleatorio por favor de cambiar

    object Configuration {
        private val neutralMode: NeutralModeValue = NeutralModeValue.Brake
        private val EXAMPLE_AMP_LIMITS = 50.0.amps

        private val motorOrientation = InvertedValue.CounterClockwise_Positive


        private val controlGains = ControlGains(
            2.0,
            0.0,
            0.1
        )

        val angularTargets = AngularMotionTargets(
            0.0.degreesPerSecond,
            0.1.seconds,
            0.1.seconds
        )


        val motorConfig: TalonFXConfiguration = KrakenMotors.createTalonFXConfiguration(
            Optional.of(KrakenMotors.configureMotorOutputs(neutralMode, motorOrientation)),
            Optional.of(KrakenMotors.configureCurrentLimits(EXAMPLE_AMP_LIMITS, false, 0.0.amps)),
            Optional.of(KrakenMotors.configureSlot0(controlGains)),Optional.of(KrakenMotors.configureAngularMotionMagic(angularTargets,
                PhysicalLimits.REDUCTION)))

        val motionMagicRequest = MotionMagicVoltage(0.0)

    }
    }
