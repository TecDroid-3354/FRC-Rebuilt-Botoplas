package frc.robot.subsystems.intake
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import frc.template.utils.amps
import frc.template.utils.controlProfiles.AngularMotionTargets
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.NumericId
import frc.template.utils.devices.RotationalDirection
import frc.template.utils.mechanical.Reduction
import frc.template.utils.radians
import frc.template.utils.seconds


object IntakeConstants {

    // CAN IDs
    const val kPositionMotorId = 20
    const val kRollerMotorId = 21
    const val kPositionMotorFollowerID = 22


    // Motor configuration
    val PostionMotor = RotationalDirection.Counterclockwise
    val RollMotor = RotationalDirection.Counterclockwise


    // Current limit in amps
    val PostionMotorLimitAmps = 40.0.amps
    val RollerMotorLimitAmps = 40.0.amps


    // Absolute encoder configuration


    val encoder= NumericId (1)

    // Offset applied to absolute encoder reading
    val kAbsoluteEncoderOffset: Angle = Units.Rotations.of(0.0)

    // Intake Positions (mechanism side)

    val kDeployedPosition: Angle = Units.Rotations.of(1.2)
    val kRetractedPosition: Angle = Units.Rotations.of(0.0)


    // -------------------------------
// Physical limits (mechanism side)
// -------------------------------
    val absoluteMaximum: Angle = Units.Rotations.of(0.0)
    val absoluteMinumum: Angle = Units.Rotations.of(1.3)

    // -------------------------------
// Reduction (mechanism -> motor)
// -------------------------------
    val kReduction = Reduction(1.0 / 2.0)


    // -------------------------------
// Control gains
// -------------------------------

    val controlGains = ControlGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    val angularMotionTargets: AngularMotionTargets = AngularMotionTargets(
        cruiseVelocity = 0.0.radians.per(Units.Seconds),
        accelerationTimePeriod = 0.0.seconds,
        jerkTimePeriod = 0.0.seconds
    )



    // -------------------------------
// Roller voltage
// -------------------------------
    val kRollerVoltage: Voltage = Units.Volts.of(8.0)
}
