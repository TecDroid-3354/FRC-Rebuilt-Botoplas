package frc.robot.subsystems.intake

import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.shooter.IntakeConstants
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.devices.ThroughBoreAbsoluteEncoder
import frc.template.utils.rotations
import org.littletonrobotics.junction.AutoLogOutput
import java.util.Optional
import kotlin.math.abs

/**
 *
 */
private enum class IntakePositions(val pose: Angle) {
    RETRACTED(IntakeConstants.RetractilePositions.RetractedPose),
    DEPLOYED(IntakeConstants.RetractilePositions.DeployedPose)
}

class Intake() : SubsystemBase() {
    // ---------------------------------------
    // PRIVATE — Deployable Motors Declaration
    // ---------------------------------------
    private val leadDeployableMotor     : OpTalonFX =
        OpTalonFX(IntakeConstants.Identification.LEAD_DEPLOY_MOTOR_ID)
    private val followerDeployableMotor : OpTalonFX =
        OpTalonFX(IntakeConstants.Identification.FOLLOWER_DEPLOY_MOTOR_ID)

    // -----------------------------------
    // PRIVATE — Intake motors Declaration
    // -----------------------------------
    private val rollersMotorController  : OpTalonFX =
        OpTalonFX(IntakeConstants.Identification.ROLLERS_MOTOR_ID)

    // --------------------------------------
    // PRIVATE — Absolute Encoder declaration
    // --------------------------------------
    private val absoluteEncoder         : ThroughBoreAbsoluteEncoder =
        ThroughBoreAbsoluteEncoder(
            IntakeConstants.Identification.ABSOLUTE_ENCODER_ID,
            IntakeConstants.Configuration.AbsoluteEncoder.offset,
            IntakeConstants.Configuration.AbsoluteEncoder.inverted,
            IntakeConstants.Configuration.AbsoluteEncoder.brand,
            Optional.empty())

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var targetPose: IntakePositions = IntakePositions.RETRACTED

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val leadDeployableConnectedAlert: Alert =
        Alert(IntakeConstants.Telemetry.INTAKE_CONNECTED_ALERTS_FIELD,
        "Intake Deploy Component Motor ID ${IntakeConstants.Identification.LEAD_DEPLOY_MOTOR_ID} Disconnected",
        Alert.AlertType.kError)

    private val followerDeployableConnectedAlert: Alert =
        Alert(IntakeConstants.Telemetry.INTAKE_CONNECTED_ALERTS_FIELD,
        "Intake Deploy Component Motor ID ${IntakeConstants.Identification.FOLLOWER_DEPLOY_MOTOR_ID} Disconnected",
        Alert.AlertType.kError)

    private val rollersComponentConnectedAlert: Alert =
        Alert(IntakeConstants.Telemetry.INTAKE_CONNECTED_ALERTS_FIELD,
        "Intake Rollers Component Motor ID ${IntakeConstants.Identification.ROLLERS_MOTOR_ID} Disconnected",
        Alert.AlertType.kError)

    private val absoluteEncoderConnectedAlert: Alert =
        Alert(IntakeConstants.Telemetry.INTAKE_CONNECTED_ALERTS_FIELD,
            "Intake Absolute Encoder ID ${IntakeConstants.Identification.ABSOLUTE_ENCODER_ID} Disconnected",
            Alert.AlertType.kError)


    /**
     * Called upon [frc.robot.subsystems.intake.Intake] creation. Used to call motors config method
     * and match the motor's encoder position to the absolute encoder position.
     */
    init {
        configurePositionMotor()
        matchRelativeToAbsolute()
        leadDeployableConnectedAlert.set(leadDeployableMotor.isConnected.invoke().not())
        followerDeployableConnectedAlert.set(followerDeployableMotor.isConnected.invoke().not())
        rollersComponentConnectedAlert.set(rollersMotorController.isConnected.invoke().not())
        absoluteEncoderConnectedAlert.set(absoluteEncoder.isConnected.not())
    }

    /**
     * Called every 20ms loop. Used to update alerts.
     * TODO() = Update deployable component motors' PID through here.
     */
    override fun periodic() {
        leadDeployableConnectedAlert.set(leadDeployableMotor.isConnected.invoke().not())
        followerDeployableConnectedAlert.set(followerDeployableMotor.isConnected.invoke().not())
        rollersComponentConnectedAlert.set(rollersMotorController.isConnected.invoke().not())
        absoluteEncoderConnectedAlert.set(absoluteEncoder.isConnected.not())
    }

    // ----------------------------------------------
    // PRIVATE — Runnable Motors Control — Deployable
    // ----------------------------------------------

    /**
     * Calls a [OpTalonFX.positionRequestSubsystem] in order to clamp the requested angle between the
     * [IntakeConstants.PhysicalLimits.Limits] set. It also considers reduction,
     * so it's just necessary to give it to the method.
     * @param angle The desired intake position.
     */
    private fun setPosition(angle: Angle) {
        leadDeployableMotor.positionRequestSubsystem(
            angle,
            IntakeConstants.PhysicalLimits.Limits,
            IntakeConstants.PhysicalLimits.Reduction
        )
    }

    // -------------------------------------------
    // PRIVATE — Runnable Motors Control — Rollers
    // -------------------------------------------

    /**
     * Sets a desired [Voltage] to the intake's rollers motor controller
     * @param voltage the desired voltage to be applied
     */
    private fun setVoltage(voltage: Voltage) {
        rollersMotorController.voltageRequest(voltage)
    }

    // -----------------------------------------
    // PRIVATE — CMD Motors Control — Deployable
    // -----------------------------------------

    /**
     * Sets a desired [IntakePositions] which holds a known position for either retracted o deployed position.
     * Keeps track of the current requested position and assigns it to [targetPose].
     * @param pose the desired pose to go to
     * @return A command requesting the given [IntakePositions] and then waiting until the error minimizes for
     * precise control.
     */
    private fun setPositionCMD(pose: IntakePositions): Command {
        targetPose = pose
        return InstantCommand({ setPosition(pose.pose) }, this)
    }

    // ---------------------------------
    // PRIVATE — CMD Motors Control — Intake
    // ---------------------------------

    /**
     * Calls [setVoltage] in order to set an output to the roller's controller
     * @return a command calling the voltage method
     */
    private fun setVoltageCMD(voltage: Voltage): Command {
        return InstantCommand({ setVoltage(voltage) }, this)
    }

    // ---------------------------------
    // PUBLIC — CMD Subsystem control
    // ---------------------------------

    /**
     * Deploys the intake and then enables the rollers through a [SequentialCommandGroup]
     * @return A sequential command group that sets a pose and enables the rollers
     */
    fun deployCMD(): Command {
        return SequentialCommandGroup(
            setPositionCMD(IntakePositions.DEPLOYED),
            WaitUntilCommand { getDeployableError()
                .lte(IntakeConstants.PhysicalLimits.DeployablePositionDelta) },
            setVoltageCMD(IntakeConstants.VoltageTargets.EnabledRollersVoltage)
        )
    }


    /**
     * Retracts the intake and then enables the rollers through a [SequentialCommandGroup]
     * @return A sequential command group that sets a pose and disables the rollers
     */
    fun retractCMD(): Command {
        return SequentialCommandGroup(
            setVoltageCMD(IntakeConstants.VoltageTargets.DisabledRollersVoltage),
            setPositionCMD(IntakePositions.RETRACTED)
        )
    }

    // ---------------------------------
    // PRIVATE — Matching absolute encoder to motor's relative encoder
    // ---------------------------------

    /**
     * Literally matches the absolute encoder position to the [leadDeployableMotor] to always ensure its set angles
     * are within the correct [frc.template.utils.safety.MeasureLimits]
     */
    private fun matchRelativeToAbsolute() {
        val absolutePosition = absoluteEncoder.position

        leadDeployableMotor.getMotorInstance().setPosition(absolutePosition)
    }

    // ---------------------------------
    // PUBLIC — Telemetry methods
    // ---------------------------------

    /**
     * The current deployable component position.
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable motor's current angle
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_ANGLE_FIELD)
    private fun getDeployableIntakeAngle(): Angle {
        return IntakeConstants.PhysicalLimits.Reduction.apply(leadDeployableMotor.position.value)
    }

    /**
     * The current voltage of the rollers component.
     * This voltage can be seen live in the "Intake" tab of AdvantageScope.
     * @return the roller's motor voltage
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_VOLTAGE_FIELD)
    private fun getRollersVoltage(): Voltage {
        return rollersMotorController.outputVoltage.value
    }

    /**
     * Gets the subsystem error by subtracting the current [targetPose] to the actual motor's angle reading,
     * after transforming it to a subsystem angle.
     * This error can be seen live in the "Intake" tab of AdvantageScope.
     * @return The error between the deployable component [targetPose] and its current position.
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_DEPLOYABLE_ERROR)
    private fun getDeployableError(): Angle {
        return abs(
            targetPose.pose.`in`(Units.Rotations).minus(
                IntakeConstants.PhysicalLimits.Reduction.apply(
                    leadDeployableMotor.position.value.`in`(Units.Rotations)
                )
            )
        ).rotations
    }

    // -------------------------------
    // PUBLIC — Neutral Mode control
    // -------------------------------

    /**
     * Sets the deployable motors [com.ctre.phoenix6.signals.NeutralModeValue] to coast. Subsystem set as requirement.
     * @return an [InstantCommand] requesting motor coast.
     */
    fun coastCMD(): Command {
        return InstantCommand({
            leadDeployableMotor.coast()
            followerDeployableMotor.coast()
                              }, this)
    }

    /**
     * Sets the deployable motors [com.ctre.phoenix6.signals.NeutralModeValue] to brake. Subsystem set as requirement.
     * @return an [InstantCommand] requesting motor brake.
     */
    fun brakeCMD(): Command {
        return InstantCommand({
            leadDeployableMotor.brake()
            followerDeployableMotor.brake()
        }, this)
    }

    // -------------------------------
    // Motor configuration (Phoenix 6)
    // -------------------------------
    private fun configurePositionMotor() {
        leadDeployableMotor.applyConfigAndClearFaults(IntakeConstants.Configuration.motorsConfig)
        leadDeployableMotor.applyConfigAndClearFaults(IntakeConstants.Configuration.motorsConfig)

        followerDeployableMotor.follow(leadDeployableMotor.getMotorInstance(), MotorAlignmentValue.Aligned)
    }
}
