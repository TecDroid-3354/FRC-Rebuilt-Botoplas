package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.Slot0Configs
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
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.devices.ThroughBoreAbsoluteEncoder
import frc.template.utils.rotations
import frc.template.utils.volts
import org.littletonrobotics.junction.AutoLogOutput
import java.util.Optional
import kotlin.math.abs

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
    private var targetAngle: Angle = IntakeConstants.RetractileAngles.RetractedAngle

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
        configureMotors()
        matchRelativeToAbsolute()
    }

    /**
     * Called every 20ms loop. Used to update alerts and keep track of changes in PIDF and voltage targets values.
     */
    override fun periodic() {
        leadDeployableConnectedAlert.set(leadDeployableMotor.isConnected.invoke().not())
        followerDeployableConnectedAlert.set(followerDeployableMotor.isConnected.invoke().not())
        rollersComponentConnectedAlert.set(rollersMotorController.isConnected.invoke().not())
        absoluteEncoderConnectedAlert.set(absoluteEncoder.isConnected.not())

        if (IntakeConstants.Tunables.motorkP.hasChanged(hashCode())
            || IntakeConstants.Tunables.motorkI.hasChanged(hashCode())
            || IntakeConstants.Tunables.motorkD.hasChanged(hashCode())
            || IntakeConstants.Tunables.motorkF.hasChanged(hashCode())) {
            updateDeployableMotorsPIDF(
                IntakeConstants.Tunables.motorkP.get(), IntakeConstants.Tunables.motorkI.get(),
                IntakeConstants.Tunables.motorkD.get(), IntakeConstants.Tunables.motorkF.get()
            )
        }

        if (IntakeConstants.Tunables.enabledRollersVoltage.hasChanged(hashCode())
            || IntakeConstants.Tunables.idleRollersVoltage.hasChanged(hashCode())) {
            updateRollersTargetVoltage(
                IntakeConstants.Tunables.enabledRollersVoltage.get(),
                IntakeConstants.Tunables.idleRollersVoltage.get()
            )
        }
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
     * Keeps track of the current requested position and assigns it to [targetAngle].
     * @param pose the desired pose to go to
     * @return A command requesting the given [IntakePositions] and then waiting until the error minimizes for
     * precise control.
     */
    private fun setPositionCMD(pose: Angle): Command {
        targetAngle = pose
        return InstantCommand({ setPosition(pose) }, this)
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
    fun enableIntakeCMD(): Command {
        return SequentialCommandGroup(
            setPositionCMD(IntakeConstants.RetractileAngles.DeployedAngle),
            WaitUntilCommand { getDeployableError()
                .lte(IntakeConstants.PhysicalLimits.DeployableAngleDelta) },
            setVoltageCMD(IntakeConstants.VoltageTargets.EnabledRollersVoltage)
        )
    }


    /**
     * Retracts the intake and then enables the rollers through a [SequentialCommandGroup]
     * @return A sequential command group that sets a pose and disables the rollers
     */
    fun disableIntakeCMD(): Command {
        return SequentialCommandGroup(
            setVoltageCMD(IntakeConstants.VoltageTargets.IdleRollersVoltage),
            setPositionCMD(IntakeConstants.RetractileAngles.RetractedAngle)
        )
    }

    fun setDeployableAngleOnly(angle: Angle): Command {
        return setPositionCMD(angle)
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
     * @return the deployable component current angle
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_ANGLE_FIELD)
    private fun getDeployableIntakeAngle(): Angle {
        return IntakeConstants.PhysicalLimits.Reduction.apply(leadDeployableMotor.position.value)
    }

    /**
     * The target deployable component position.
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable component target angle
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_TARGET_ANGLE_FIELD)
    private fun getDeployableIntakeTargetAngle(): Angle {
        return targetAngle
    }

    /**
     * Gets the subsystem error by subtracting the current [targetAngle] to the actual motor's angle reading,
     * after transforming it to a subsystem angle.
     * This error can be seen live in the "Intake" tab of AdvantageScope.
     * @return The error between the deployable component [targetAngle] and its current position.
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_DEPLOYABLE_ERROR)
    fun getDeployableError(): Angle {
        return abs(
            targetAngle.`in`(Units.Rotations).minus(
                IntakeConstants.PhysicalLimits.Reduction.apply(
                    leadDeployableMotor.position.value.`in`(Units.Rotations)
                )
            )
        ).rotations
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
    private fun configureMotors() {
        leadDeployableMotor.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig)
        leadDeployableMotor.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig)

        followerDeployableMotor.follow(leadDeployableMotor.getMotorInstance(), MotorAlignmentValue.Aligned)

        rollersMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig)
    }

    /**
     * Used to update the PIDF of the deployable component motors. Initial configuration remains the same, only [Slot0Configs] regarding
     * kP, kI, kD and kF are changed depending on the value passed to the [frc.robot.utils.controlProfiles.LoggedTunableNumber]
     * @param kP P coefficient received live
     * @param kI I coefficient received live
     * @param kD D coefficient received live
     * @param kF F coefficient received live
     */
    private fun updateDeployableMotorsPIDF(kP: Double, kI: Double, kD: Double, kF: Double) {
        val newSlot0Configs: Slot0Configs = KrakenMotors.configureSlot0(
            ControlGains(
                kP, kI, kD, kF,
                IntakeConstants.Configuration.controlGains.s,
                IntakeConstants.Configuration.controlGains.v,
                IntakeConstants.Configuration.controlGains.a,
                IntakeConstants.Configuration.controlGains.g
            )
        )

        leadDeployableMotor.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig.withSlot0(newSlot0Configs))
        followerDeployableMotor.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig.withSlot0(newSlot0Configs))
    }

    /**
     * Used to update the voltage targets of the rollers component.
     * @param enabledVoltage Voltage target when active. Received live
     * @param idleVoltage Voltage target when idle. Received live
     */
    private fun updateRollersTargetVoltage(enabledVoltage: Double, idleVoltage: Double) {
        IntakeConstants.VoltageTargets.EnabledRollersVoltage = enabledVoltage.volts
        IntakeConstants.VoltageTargets.IdleRollersVoltage = idleVoltage.volts
    }
}
