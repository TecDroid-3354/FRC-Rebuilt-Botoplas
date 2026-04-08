package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.Slot0Configs
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.RobotConstants
import frc.robot.subsystems.shooter.IntakeConstants
import frc.robot.utils.subsystemUtils.generic.SysIdSubsystem
import frc.robot.utils.subsystemUtils.identification.SysIdRoutines
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.meters
import frc.template.utils.seconds
import frc.template.utils.volts
import org.littletonrobotics.junction.AutoLogOutput
import java.util.function.Supplier
import kotlin.math.abs

enum class IntakePositions {
    DEPLOYED, CLUSTERED;
}

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
    private var targetDisplacement: Distance = IntakeConstants.RetractileAngles.ClusteredDisplacement

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
    override val sysIdForwardRunningCondition: () -> Boolean = { getDeployableDisplacementMeters() < IntakeConstants.PhysicalLimits.DeployableLimits.maximum.`in`(Units.Meters) }
    override val sysIdBackwardRunningCondition: () -> Boolean = { getDeployableDisplacementMeters() > IntakeConstants.PhysicalLimits.DeployableLimits.minimum.`in`(Units.Meters) }

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

        if (IntakeConstants.Tunables.enabledRollersVoltage.hasChanged(hashCode())
            || IntakeConstants.Tunables.clusteringRollersVoltage.hasChanged(hashCode())
            || IntakeConstants.Tunables.idleRollersVoltage.hasChanged(hashCode())) {
            updateRollersTargetVelocity(
                IntakeConstants.Tunables.enabledRollersVoltage.get(),
                IntakeConstants.Tunables.clusteringRollersVoltage.get(),
                IntakeConstants.Tunables.idleRollersVoltage.get()
            )
        }
    }

    // ----------------------------------------------
    // PRIVATE — Runnable Motors Control — Deployable
    // ----------------------------------------------

    /**
     * Calls a [OpTalonFX.positionRequestSubsystem] in order to clamp the requested displacement between the
     * [IntakeConstants.PhysicalLimits.DeployableLimits] set. It also considers reduction,
     * so it's just necessary to give it to the method.
     * @param displacement The desired intake displacement.
     */
    private fun setPosition(displacement: Distance) {
        targetDisplacement = IntakeConstants.PhysicalLimits.DeployableLimits.coerceIn(displacement) as Distance
        deployableMotorController.positionRequestSubsystem(
            targetDisplacement,
            IntakeConstants.PhysicalLimits.DeployableLimits,
            IntakeConstants.PhysicalLimits.DeployableReduction,
            IntakeConstants.Configuration.deployableMotorSprocket
        )
    }

    /**
     * Common method for setting voltage to the deployable controller.
     * Usable only when running a [sysIdRoutines]
     * @param voltage The SysId-requested voltage.
     */
    override fun setVoltage(voltage: Voltage) {
        deployableMotorController.voltageRequest(voltage)
    }

    // -------------------------------------------
    // PRIVATE — Runnable Motors Control — Rollers
    // -------------------------------------------

    /**
     * Sets a desired [AngularVelocity] to the intake's rollers motor controller
     * @param voltage The desired velocity to be applied
     */
    fun setRollersVoltage(voltage: Voltage) {
        rollersLeadMotorController.voltageRequest(voltage)
    }

    // -----------------------------------------
    // PRIVATE — CMD Motors Control — Deployable
    // -----------------------------------------


    /**
     * Sets a desired [IntakePositions] which holds a known position for either retracted o deployed position.
     * Keeps track of the current requested position and assigns it to [targetDisplacement].
     * @param position the desired pose to go to
     * @return A command requesting the given [IntakePositions].
     */
    private fun setPositionWithDisplacementCMD(position: Distance): Command {
        return InstantCommand({ setPosition(position) }, this)
    }

    /**
     * Intended to change the [com.ctre.phoenix6.configs.MotionMagicConfigs] depending on the requested position.
     * Re-configuration of the motor is necessary, since changing [com.ctre.phoenix6.configs.MotionMagicConfigs]
     * on-the-fly requires [com.ctre.phoenix6.controls.DynamicMotionMagicVoltage], which is a Pro Licensed feature.
     * - CLUSTERED is intended to use while shooting, hence it must be slower.
     * - DEPLOYED is used with normal velocity.
     * @param position The requested position.
     */
    private fun setPositionCMD(position: IntakePositions): Command {
        return when (position) {
            IntakePositions.CLUSTERED ->
                SequentialCommandGroup(
                    InstantCommand({ deployableMotorController.applyConfigAndClearFaults(
                            IntakeConstants.Configuration.deployableMotorsConfig.withMotionMagic(
                                IntakeConstants.Configuration.clusteringMotionMagic
                            )
                        )
                    }),
                    WaitCommand(0.08.seconds),
                    setPositionWithDisplacementCMD (IntakeConstants.RetractileAngles.ClusteredDisplacement)
                )

            IntakePositions.DEPLOYED ->
                SequentialCommandGroup(
                    InstantCommand({ deployableMotorController.applyConfigAndClearFaults(
                        IntakeConstants.Configuration.deployableMotorsConfig
                        )
                    }),
                    WaitCommand(0.08.seconds),
                    setPositionWithDisplacementCMD (IntakeConstants.RetractileAngles.DeployedDisplacement)
                )
        }
    }

    // ---------------------------------
    // PRIVATE — CMD Motors Control — Intake
    // ---------------------------------

    /**
     * Calls [setRollersVoltage] in order to set an output to the roller's controller
     * @return a command calling the voltage method
     */
    private fun setRollersVoltageCMD(voltage: Voltage): Command {
        return InstantCommand({ setRollersVoltage(voltage) }, this)
    }

    // ---------------------------------
    // PUBLIC — CMD Subsystem control
    // ---------------------------------

    fun setRollersVoltageCMD(): Command {
        return InstantCommand({ rollersLeadMotorController.voltageRequest(10.0.volts) }, this)
    }

    fun stopRollersVoltageCMD(): Command {
        return InstantCommand({ rollersLeadMotorController.voltageRequest(0.0.volts) }, this)
    }

    fun setRollersVoltageTunableCMD(voltage: Supplier<Voltage>): Command {
        return RunCommand({ setRollersVoltage(voltage.get()) }, this )
    }
    /**
     * Deploys the intake and then enables the rollers through a [SequentialCommandGroup]
     * @return A sequential command group that sets a pose and enables the rollers
     */
    fun deployAndEnableIntakeCMD(): Command {
        return SequentialCommandGroup(
            setPositionCMD(IntakePositions.DEPLOYED),
            WaitUntilCommand { getDeployableError()
                .lte(IntakeConstants.RetractileAngles.DeployableDisplacementDelta) },
            setRollersVoltageCMD(IntakeConstants.VoltageTargets.EnabledRollersVoltage)
        )
    }

    /**
     * Deploys the intake and then disables the rollers through a [SequentialCommandGroup]
     * @return A sequential command group that sets a pose and enables the rollers
     */
    fun deployAndDisableIntakeCMD(): Command {
        return SequentialCommandGroup(
            setPositionCMD(IntakePositions.DEPLOYED),
            disableRollersCMD()
        )
    }

    fun clusterIntakeCMD(): Command {
        return SequentialCommandGroup(
            InstantCommand({ setRollersVoltage(IntakeConstants.VoltageTargets.ClusteringRollersVoltage) }),
            //WaitCommand(0.1.seconds),
            setPositionCMD(IntakePositions.CLUSTERED),
        )
    }

    /**
     * Sets the [IntakeConstants.VoltageTargets.IdleRollersVoltage] velocity to the rollers through. Subsystem is set as requirement.
     * @return An [InstantCommand] that sets a pose and disables the rollers
     */
    fun disableRollersCMD(): Command {
        return InstantCommand({ setRollersVoltage(0.0.volts) }, this)
    }

    fun stopMotor(): Command {
        return InstantCommand({ deployableMotorController.stopMotor() }, this)
    }

    fun setDeployableDisplacementOnly(displacement: Distance): Command {
        return ParallelCommandGroup(
            InstantCommand({ setPosition(displacement) }),
            disableRollersCMD()
        )
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
    // PRIVATE — Telemetry methods
    // ---------------------------------

    /**
     * The current deployable component position (in motor rotations).
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable component current angle
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_MOTOR_ANGLE_FIELD)
    private fun getDeployableAngleMotor(): Angle {
        return deployableMotorController.getPosition()
    }

    /**
     * The target deployable component position (in motor rotations).
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable component target angle
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_MOTOR_TARGET_ANGLE_FIELD)
    private fun getDeployableTargetAngleMotor(): Angle {
        return IntakeConstants.PhysicalLimits.DeployableReduction.unapply(
            IntakeConstants.Configuration.deployableMotorSprocket.linearDisplacementToAngularDisplacement(
                targetDisplacement
            )
        )
    }

    /**
     * The current deployable component position.
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable component current displacement
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_DISPLACEMENT_FIELD, unit = "meters")
    private fun getDeployableDisplacementMeters(): Double {
        return IntakeConstants.PhysicalLimits.DeployableReduction.apply(
            IntakeConstants.Configuration.deployableMotorSprocket.angularDisplacementToLinearDisplacement(
                deployableMotorController.getPosition()
            )
        ).`in`(Units.Meters)
    }

    /**
     * The target deployable component position.
     * This position can be seen live in the "Intake" tab of AdvantageScope.
     * @return the deployable component target displacement
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_TARGET_DISPLACEMENT_FIELD, unit = "meters")
    private fun getDeployableIntakeTargetDisplacementMeters(): Double {
        return targetDisplacement.`in`(Units.Meters)
    }

    /**
     * Gets the subsystem error by subtracting the current displacement from the [targetDisplacement],
     * after transforming the motor reading to subsystem displacement.
     * This error can be seen live in the "Intake" tab of AdvantageScope.
     * @return The error between the deployable component [targetDisplacement] and its current position.
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_DEPLOYABLE_ERROR)
    fun getDeployableError(): Distance {
        return abs(
            targetDisplacement.`in`(Units.Meters).minus(getDeployableDisplacementMeters())
        ).meters
    }

    /**
     * The current RPMs of the rollers component. The motor reports in RPS, hence the .times(60.0)
     * This voltage can be seen live in the "Intake" tab of AdvantageScope.
     * @return the roller's motor velocity.
     */
    @AutoLogOutput(key = IntakeConstants.Telemetry.INTAKE_RPM_FIELD)
    private fun getRollersVelocity(): AngularVelocity {
        return rollersLeadMotorController.getVelocity().times(60.0)
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

    /**
     * Clears faults and applies the corresponding configuration to both the deployable and rollers motors.
     * Follower Control is also applied to the corresponding rollers motor.
     */
    private fun configureMotors() {
        deployableMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.deployableMotorsConfig)

        rollersLeadMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig)
        rollersFollowerMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.rollerMotorConfig)

        rollersFollowerMotorController.follow(rollersLeadMotorController.getMotorInstance(),
            IntakeConstants.Configuration.rollerFollowerAlignment)
    }

    /**
     * Used to update the PIDF of the deployable component motor. Initial configuration remains the same, only [Slot0Configs] regarding
     * kP, kI, kD and kF are changed depending on the value passed to the [frc.robot.utils.controlProfiles.LoggedTunableNumber]
     * @param kP P coefficient received live through AdvantageScope or Elastic.
     * @param kI I coefficient received live through AdvantageScope or Elastic.
     * @param kD D coefficient received live through AdvantageScope or Elastic.
     * @param kF F coefficient received live through AdvantageScope or Elastic.
     */
    private fun updateDeployableMotorsPIDF(kP: Double, kI: Double, kD: Double, kF: Double) {
        val newSlot0Configs: Slot0Configs = KrakenMotors.configureSlot0(
            ControlGains(
                kP, kI, kD, kF,
                IntakeConstants.Configuration.deployControlGains.s,
                IntakeConstants.Configuration.deployControlGains.v,
                IntakeConstants.Configuration.deployControlGains.a,
                IntakeConstants.Configuration.deployControlGains.g
            )
        )

        deployableMotorController.applyConfigAndClearFaults(IntakeConstants.Configuration.deployableMotorsConfig.withSlot0(newSlot0Configs))
    }

    /**
     * Used to update the RPS targets of the rollers component.
     * @param enabledVoltage Target RPS when active. Received live through AdvantageScope or Elastic.
     * @param idleVoltage Target RPS when idle. Received live through AdvantageScope or Elastic.
     */
    private fun updateRollersTargetVelocity(enabledVoltage: Double, clusteringVoltage: Double, idleVoltage: Double) {
        IntakeConstants.VoltageTargets.EnabledRollersVoltage = enabledVoltage.volts
        IntakeConstants.VoltageTargets.ClusteringRollersVoltage = clusteringVoltage.volts
        IntakeConstants.VoltageTargets.IdleRollersVoltage = idleVoltage.volts
    }
}
