package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.subsystems.hood.HoodConstants
import frc.robot.utils.subsystemUtils.generic.SysIdSubsystem
import frc.robot.utils.subsystemUtils.identification.SysIdRoutines
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

class Climber() : SysIdSubsystem("Climber") {

    // ---------------------------------------
    // PRIVATE — Motors Declaration
    // ---------------------------------------
    private val motorController             : OpTalonFX =
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

    /// --------------------------------------------------------------------------------
    // PRIVATE SYS ID — Running Conditions, Relevant Variables and Routines Declaration
    // --------------------------------------------------------------------------------
    override val sysIdForwardRunningCondition: () -> Boolean = { getClimberAngle() < HoodConstants.PhysicalLimits.Limits.maximum }
    override val sysIdBackwardRunningCondition: () -> Boolean = { getClimberAngle() > HoodConstants.PhysicalLimits.Limits.minimum }

    /**
     * [motorPosition] holds the current motor's position without gear ratios
     */
    override val motorPosition: Angle
        get() = motorController.getPosition()
    /**
     * [motorVelocity] holds the current motor's velocity without gear ratios
     */
    override val motorVelocity: AngularVelocity
        get() = motorController.getVelocity()
    /**
     * [power] holds the current motor's power
     */
    override val power: Double
        get() = motorController.getPower()

    /**
     * [sysIdRoutines] holds the 4 possible SysId routines, later called in [sysIdQuasistaticRoutine] & [sysIdDynamicRoutine]
     */
    private val sysIdRoutines: SysIdRoutines = createIdentificationRoutines().createTests()
    
    /**
     * Called upon [frc.robot.subsystems.climber.Climber] creation. Used to call motors config method.
     */
    init {
        motorController.applyConfigAndClearFaults(ClimberConstants.Configuration.motorConfig)
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
        motorController.positionRequestSubsystem(
            angle,
            ClimberConstants.PhysicalLimits.Limits,
            ClimberConstants.PhysicalLimits.Reduction
        )
    }

    /**
     * Creates a voltage request to the motor controller. Used within the [SysIdSubsystem] interface to
     * run the characterization methods.
     */
    override fun setVoltage(voltage: Voltage) {
        motorController.voltageRequest(voltage)
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
     * Literally matches the absolute encoder position to the [motorController] to always ensure its angle
     * are within the correct [frc.template.utils.safety.MeasureLimits]
     */
    private fun matchRelativeToAbsolute() {
        val absolutePosition = encoder.position

        motorController.getMotorInstance().setPosition(absolutePosition)
    }

    // -------------------------------
    // PUBLIC — SysId Routines
    // -------------------------------

    /**
     * A Quasistatic SysId routine is scheduled and turned off immediately once [sysIdForwardRunningCondition]
     * or [sysIdBackwardRunningCondition] become false, meaning the subsystem's limits were met.
     * Quasistatic means the magnitude of the supplied voltage will gradually increase.
     * @param direction Whether to run the subsystem forward of backwards.
     * @return A [Command] with a quasistatic routine following the specified direction
     */
    fun sysIdQuasistaticRoutine(direction: SysIdRoutine.Direction): Command {
        return when (direction) {
            SysIdRoutine.Direction.kForward -> sysIdRoutines.quasistaticForward
            SysIdRoutine.Direction.kReverse -> sysIdRoutines.quasistaticBackward
        }
    }

    /**
     * A Dynamic SysId routine is scheduled and turned off immediately once [sysIdForwardRunningCondition]
     * or [sysIdBackwardRunningCondition] become false, meaning the subsystem's limits were met.
     * Dynamic means the magnitude of the supplied voltage is fixed.
     * @param direction Whether to run the subsystem forward of backwards.
     * @return A [Command] with a dynamic routine following the specified direction
     */
    fun sysIdDynamicRoutine(direction: SysIdRoutine.Direction): Command {
        return when (direction) {
            SysIdRoutine.Direction.kForward -> sysIdRoutines.dynamicForward
            SysIdRoutine.Direction.kReverse -> sysIdRoutines.dynamicBackward
        }
    }

    // ---------------------------------
    // PUBLIC — Telemetry methods
    // ---------------------------------

    /**
     * @return the climber motor's current angle
     */
    @AutoLogOutput(key = ClimberConstants.Telemetry.CLIMBER_ANGLE_FIELD)
    private fun getClimberAngle(): Angle {
        return ClimberConstants.PhysicalLimits.Reduction.apply(motorController.getPosition())
    }

    /**
     * Gets the subsystem error by subtracting the current [targetPose] to the actual motor's angle reading
     */
    @AutoLogOutput(key = ClimberConstants.Telemetry.CLIMBER_ERROR)
    private fun getClimberError(): Angle {
        return abs(
            targetPose.pose.`in`(Units.Rotations).minus(
                ClimberConstants.PhysicalLimits.Reduction.apply(
                    motorController.getPosition().`in`(Units.Rotations)
                )
            )
        ).rotations
    }
}
