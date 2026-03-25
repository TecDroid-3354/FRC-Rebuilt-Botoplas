package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.Slot0Configs
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.RobotConstants
import frc.robot.subsystems.shooter.IntakeConstants
import frc.robot.utils.subsystemUtils.generic.SysIdSubsystem
import frc.robot.utils.subsystemUtils.identification.SysIdRoutines
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.degrees
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.rotationsPerSecond
import org.littletonrobotics.junction.AutoLogOutput
import kotlin.math.abs

class Intake() : SysIdSubsystem("Intake") {
    // ---------------------------------------
    // PRIVATE — Deployable Motors Declaration
    // ---------------------------------------
    private val deployableMotorController     : OpTalonFX =
        OpTalonFX(IntakeConstants.Identification.LEAD_DEPLOY_MOTOR_ID, RobotConstants.Identification.ALTERNATE_CANBUS)

    // -----------------------------------
    // PRIVATE — Intake motors Declaration
    // -----------------------------------
    private val rollersLeadMotorController  : OpTalonFX =
        OpTalonFX(IntakeConstants.Identification.LEAD_ROLLERS_MOTOR_ID, RobotConstants.Identification.ALTERNATE_CANBUS)
    private val rollersFollowerMotorController  : OpTalonFX =
        OpTalonFX(IntakeConstants.Identification.FOLLOWER_ROLLERS_MOTOR_ID, RobotConstants.Identification.ALTERNATE_CANBUS)

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

    private val rollersLeadComponentConnectedAlert: Alert =
        Alert(IntakeConstants.Telemetry.INTAKE_CONNECTED_ALERTS_FIELD,
        "Intake Rollers Component Motor ID ${IntakeConstants.Identification.LEAD_ROLLERS_MOTOR_ID} Disconnected",
        Alert.AlertType.kError)

    private val rollersFollowerComponentConnectedAlert: Alert =
        Alert(IntakeConstants.Telemetry.INTAKE_CONNECTED_ALERTS_FIELD,
            "Intake Rollers Component Motor ID ${IntakeConstants.Identification.FOLLOWER_ROLLERS_MOTOR_ID} Disconnected",
            Alert.AlertType.kError)

    // --------------------------------------------------------------------------------
    // PRIVATE SYS ID — Running Conditions, Relevant Variables and Routines Declaration
    // --------------------------------------------------------------------------------
    override val sysIdForwardRunningCondition: () -> Boolean = { getDeployableIntakeAngleDegrees() < IntakeConstants.PhysicalLimits.Limits.maximum.`in`(Degrees) }
    override val sysIdBackwardRunningCondition: () -> Boolean = { getDeployableIntakeAngleDegrees() > IntakeConstants.PhysicalLimits.Limits.minimum.`in`(Degrees) }

    /**
     * [motorPosition] holds the current motor's position without gear ratios
     */
    override val motorPosition: Angle
        get() = deployableMotorController.getPosition()
    /**
     * [motorVelocity] holds the current motor's velocity without gear ratios
     */
    override val motorVelocity: AngularVelocity
        get() = deployableMotorController.getVelocity()
    /**
     * [power] holds the current motor's power
     */
    override val power: Double
        get() = deployableMotorController.getPower()


    /**
     * [sysIdRoutines] holds the 4 possible SysId routines, later called in [sysIdQuasistaticRoutine] & [sysIdDynamicRoutine]
     */
    private val sysIdRoutines: SysIdRoutines = createIdentificationRoutines().createTests()


    /**
     * Called upon [frc.robot.subsystems.intake.Intake] creation. Used to call motors config method
     * and match the motor's encoder position to the absolute encoder position.
     */
    init {
        configureMotors()
    }

    /**
     * Called every 20ms loop. Used to update alerts and keep track of changes in PIDF and voltage targets values.
     */
    override fun periodic() {
        leadDeployableConnectedAlert.set(deployableMotorController.getIsConnected().not())
        rollersLeadComponentConnectedAlert.set(rollersLeadMotorController.getIsConnected().not())
        rollersFollowerComponentConnectedAlert.set(rollersFollowerMotorController.getIsConnected().not())

        if (IntakeConstants.Tunables.motorkP.hasChanged(hashCode())
            || IntakeConstants.Tunables.motorkI.hasChanged(hashCode())
            || IntakeConstants.Tunables.motorkD.hasChanged(hashCode())
            || IntakeConstants.Tunables.motorkF.hasChanged(hashCode())) {
            updateDeployableMotorsPIDF(
                IntakeConstants.Tunables.motorkP.get(), IntakeConstants.Tunables.motorkI.get(),
                IntakeConstants.Tunables.motorkD.get(), IntakeConstants.Tunables.motorkF.get()
            )
        }

        if (IntakeConstants.Tunables.enabledRollersRPMs.hasChanged(hashCode())
            || IntakeConstants.Tunables.idleRollersRPMs.hasChanged(hashCode())) {
            updateRollersTargetVelocity(
                IntakeConstants.Tunables.enabledRollersRPMs.get(),
                IntakeConstants.Tunables.idleRollersRPMs.get()
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
        targetAngle = IntakeConstants.PhysicalLimits.Limits.coerceIn(angle) as Angle
        deployableMotorController.positionRequestSubsystem(
            angle,
            IntakeConstants.PhysicalLimits.Limits,
            IntakeConstants.PhysicalLimits.Reduction
        )
    }

    /**
     * Common method for setting voltage to the deployable controller.
     * Usable only when running a [sysIdRoutines]
     */
    override fun setVoltage(voltage: Voltage) {
        deployableMotorController.voltageRequest(voltage)
    }

    // -------------------------------------------
    // PRIVATE — Runnable Motors Control — Rollers
    // -------------------------------------------

    /**
     * Sets a desired [Voltage] to the intake's rollers motor controller
     * @param voltage the desired voltage to be applied
     */
    fun setRollersVelocity(velocity: AngularVelocity) {
        rollersLeadMotorController.velocityRequest(velocity)
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
        return InstantCommand({ setPosition(pose) }, this)
    }

    // ---------------------------------
    // PRIVATE — CMD Motors Control — Intake
    // ---------------------------------

    /**
     * Calls [setRollersVelocity] in order to set an output to the roller's controller
     * @return a command calling the voltage method
     */
    fun setRollersVelocityCMD(velocity: AngularVelocity): Command {
        return InstantCommand({ setRollersVelocity(velocity) }, this)
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
            setRollersVelocityCMD(IntakeConstants.RPSTargets.EnabledRollersRPS)
        )
    }

    /**
     * Retracts the intake and then enables the rollers through a [SequentialCommandGroup]
     * @return A sequential command group that sets a pose and disables the rollers
     */
    fun disableIntakeCMD(): Command {
        return ParallelCommandGroup(
            InstantCommand({ setRollersVelocity(IntakeConstants.RPSTargets.IdleRollersRPS) }),
            InstantCommand({ setPosition(IntakeConstants.RetractileAngles.DeployedAngle) })
        )
    }

    fun disableRollersCMD(): Command {
        return InstantCommand({ setRollersVelocity(IntakeConstants.RPSTargets.IdleRollersRPS) })
    }

    fun setDeployableAngleOnly(angle: Angle): Command {
        return InstantCommand({ setPosition(angle) }, this )
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
     * The current deployable component position according to the lead motor.
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable component current angle
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_ANGLE_FIELD, unit = "degrees")
    private fun getDeployableIntakeAngleDegrees(): Double {
        return IntakeConstants.PhysicalLimits.Reduction.apply(deployableMotorController.getPosition().`in`(Degrees))
    }

    /**
     * The target deployable component position.
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable component target angle
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_TARGET_ANGLE_FIELD)
    private fun getDeployableIntakeTargetAngleDegrees(): Double {
        return targetAngle.`in`(Degrees)
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
            targetAngle.`in`(Degrees).minus(
                IntakeConstants.PhysicalLimits.Reduction.apply(
                    deployableMotorController.getPosition().`in`(Degrees)
                )
            )
        ).degrees
    }

    /**
     * The current RPMs of the rollers component.
     * This voltage can be seen live in the "Intake" tab of AdvantageScope.
     * @return the roller's motor voltage
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_RPM_FIELD)
    private fun getRollersVelocity(): AngularVelocity {
        return rollersLeadMotorController.getVelocity()
    }

    // -------------------------------
    // PUBLIC — Neutral Mode control
    // -------------------------------

    /**
     * Sets the deployable motors [com.ctre.phoenix6.signals.NeutralModeValue] to coast. Subsystem set as requirement.
     * @return an [InstantCommand] requesting motor coast.
     */
    fun coastCMD(): Command {
        return InstantCommand({ deployableMotorController.coast() }, this)
    }

    /**
     * Sets the deployable motors [com.ctre.phoenix6.signals.NeutralModeValue] to brake. Subsystem set as requirement.
     * @return an [InstantCommand] requesting motor brake.
     */
    fun brakeCMD(): Command {
        return InstantCommand({ deployableMotorController.brake() }, this)
    }

    // -------------------------------
    // Motor configuration (Phoenix 6)
    // -------------------------------
    private fun configureMotors() {
        deployableMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.deployableMotorsConfig)

        rollersLeadMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig)
        rollersFollowerMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig)

        rollersFollowerMotorController.follow(rollersLeadMotorController.getMotorInstance(),
            IntakeConstants.Configuration.rollerFollowerAlignment)
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

        deployableMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.deployableMotorsConfig.withSlot0(newSlot0Configs))
    }

    /**
     * Used to update the voltage targets of the rollers component.
     * @param enabledVoltage Voltage target when active. Received live
     * @param idleVoltage Voltage target when idle. Received live
     */
    private fun updateRollersTargetVelocity(enabledVelocity: Double, idleVelocity: Double) {
        IntakeConstants.RPSTargets.EnabledRollersRPS = enabledVelocity.rotationsPerSecond
        IntakeConstants.RPSTargets.IdleRollersRPS = idleVelocity.rotationsPerSecond
    }
}
