package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.devices.ThroughBoreAbsoluteEncoder
import frc.template.utils.rotations
import org.littletonrobotics.junction.AutoLogOutput
import java.util.Optional
import kotlin.math.abs

private enum class ClimberPositions(val pose: Angle) {
    EXTENDED(ClimberConstants.ClimberPositions.ExtendedPose),
    RETRACTED(ClimberConstants.ClimberPositions.RetractedPose)
}

class Climber() : SubsystemBase() {

    // ---------------------------------------
    // PRIVATE — Motors Declaration
    // ---------------------------------------
    private val climberMotorController     : OpTalonFX =
        OpTalonFX(ClimberConstants.Identification.CLIMBER_MOTOR_ID)

    // -------------------------------
    // PRIVATE — Absolute Encoder declaration
    // -------------------------------
    private val encoder                     : ThroughBoreAbsoluteEncoder =
        ThroughBoreAbsoluteEncoder(
            ClimberConstants.Identification.ABSOLUTE_ENCODER_ID,
            ClimberConstants.Configuration.AbsoluteEncoder.offset,
            ClimberConstants.Configuration.AbsoluteEncoder.inverted,
            ClimberConstants.Configuration.AbsoluteEncoder.brand,
            Optional.empty())

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var targetPose: ClimberPositions = ClimberPositions.RETRACTED

    /**
     * Called upon [frc.robot.subsystems.climber.Climber] creation. Used to call motors config method.
     */
    init {
        climberMotorController.applyConfigAndClearFaults(ClimberConstants.Configuration.motorConfig)
        matchRelativeToAbsolute()
    }

    // ----------------------------------------------
    // PRIVATE — Runnable Motors Control
    // ----------------------------------------------

    /**
     * Calls a [OpTalonFX.positionRequestSubsystem] in order to clamp the requested angle between the
     * [ClimberConstants.PhysicalLimits.Limits] set. It also considers reduction, so it's just necessary to give it to the method.
     * @param angle The desired climber position.
     */
    private fun setPosition(angle: Angle) {
        climberMotorController.positionRequestSubsystem(
            angle,
            ClimberConstants.PhysicalLimits.Limits,
            ClimberConstants.PhysicalLimits.Reduction
        )
    }

    // -----------------------------------------
    // PRIVATE — CMD Motors Control
    // -----------------------------------------

    /**
     * Sets a desired [ClimberPositions] which holds a known position for either retracted o extended position.
     * Keeps track of the current requested position and assigns it to [targetPose].
     * @param pose the desired pose to go to
     * @return A command requesting the given [ClimberPositions] and then waiting until the error minimizes for
     * precise control.
     */
    private fun setPositionCMD(pose: ClimberPositions): Command {
        targetPose = pose
        return InstantCommand({ setPosition(pose.pose) }, this)
    }

    // ---------------------------------
    // PUBLIC — CMD Subsystem control
    // ---------------------------------

    /**
     * @return A command that extends the climber calls the [ClimberConstants.ClimberPositions.ExtendedPose]
     */
    fun extendCMD(): Command {
       return setPositionCMD(ClimberPositions.EXTENDED)
    }

    /**
     * @return A command that retracts the climber calls the [ClimberConstants.ClimberPositions.RetractedPose]
     */
    fun retractCMD(): Command {
        return setPositionCMD(ClimberPositions.RETRACTED)
    }

    // ---------------------------------
    // PRIVATE — Matching absolute encoder to motor's relative encoder
    // ---------------------------------

    /**
     * Literally matches the absolute encoder position to the [climberMotorController] to always ensure its angle
     * are within the correct [frc.template.utils.safety.MeasureLimits]
     */
    private fun matchRelativeToAbsolute() {
        val absolutePosition = encoder.position

        climberMotorController.getMotorInstance().setPosition(absolutePosition)
    }

    // ---------------------------------
    // PUBLIC — Telemetry methods
    // ---------------------------------

    /**
     * @return the climber motor's current angle
     */
    @AutoLogOutput(key = ClimberConstants.Telemetry.CLIMBER_ANGLE_FIELD)
    private fun getDeployableIntakeAngle(): Angle {
        return ClimberConstants.PhysicalLimits.Reduction.apply(climberMotorController.position.value)
    }

    /**
     * Gets the subsystem error by subtracting the current [targetPose] to the actual motor's angle reading
     */
    @AutoLogOutput(key = ClimberConstants.Telemetry.CLIMBER_ERROR)
    private fun getClimberError(): Angle {
        return abs(
            targetPose.pose.`in`(Units.Rotations).minus(
                ClimberConstants.PhysicalLimits.Reduction.apply(
                    climberMotorController.position.value.`in`(Units.Rotations)
                )
            )
        ).rotations
    }
}
