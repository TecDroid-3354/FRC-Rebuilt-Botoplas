package frc.robot.subsystems.hood
import com.ctre.phoenix6.configs.Slot0Configs
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.utils.subsystemUtils.generic.SysIdSubsystem
import frc.robot.utils.subsystemUtils.identification.SysIdRoutines
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.OpTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import java.util.function.Supplier

class Hood() : SysIdSubsystem("Hood") {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val motorController     : OpTalonFX =
        OpTalonFX(HoodConstants.Identification.HOOD_MOTOR_ID)

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var targetAngle          : Angle = Degrees.zero()

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val hoodConnectedAlert  : Alert =
        Alert(HoodConstants.Telemetry.HOOD_CONNECTED_ALERTS_FIELD,
            "Hood Motor ID ${HoodConstants.Identification.HOOD_MOTOR_ID} Disconnected",
            Alert.AlertType.kError)

    // --------------------------------------------------------------------------------
    // PRIVATE SYS ID — Running Conditions, Relevant Variables and Routines Declaration
    // --------------------------------------------------------------------------------
    override val sysIdForwardRunningCondition: () -> Boolean = { getHoodAngle() < HoodConstants.PhysicalLimits.Limits.maximum }
    override val sysIdBackwardRunningCondition: () -> Boolean = { getHoodAngle() > HoodConstants.PhysicalLimits.Limits.minimum }

    /**
     * [motorPosition] holds the current motor's position without gear ratios
     */
    override val motorPosition: Angle
        get() = motorController.position.value
    /**
     * [motorVelocity] holds the current motor's velocity without gear ratios
     */
    override val motorVelocity: AngularVelocity
        get() = motorController.velocity.value
    /**
     * [power] holds the current motor's power
     */
    override val power: Double
        get() = motorController.power.invoke()

    /**
     * [sysIdRoutines] holds the 4 possible SysId routines, later called in [sysIdQuasistaticRoutine] & [sysIdDynamicRoutine]
     */
    private val sysIdRoutines: SysIdRoutines = createIdentificationRoutines().createTests()

    /**
     * Called upon [frc.robot.subsystems.hood.Hood] creation. Used to configure motors.
     */
    init {
        motorController.applyConfigAndClearFaults(HoodConstants.Configuration.motorConfig)
    }

    /**
     * Called every 20ms loop. Used to update alerts and keep track of changes in PIDF values.
     */
    override fun periodic() {
        hoodConnectedAlert.set(motorController.isConnected.invoke().not())

        if (HoodConstants.Tunables.motorkP.hasChanged(hashCode())
            || HoodConstants.Tunables.motorkI.hasChanged(hashCode())
            || HoodConstants.Tunables.motorkD.hasChanged(hashCode())
            || HoodConstants.Tunables.motorkF.hasChanged(hashCode())) {
            updateHoodMotorPIDF(
                HoodConstants.Tunables.motorkP.get(), HoodConstants.Tunables.motorkI.get(),
                HoodConstants.Tunables.motorkD.get(), HoodConstants.Tunables.motorkF.get()
            )
        }
    }

    // ---------------------------------
    // PRIVATE — Runnable Motors Control
    // ---------------------------------

    /**
     * Takes the desired [frc.robot.subsystems.hood.Hood] angle and transforms it to a motor position.
     * [com.ctre.phoenix6.controls.MotionMagicVoltage] is used to request the position.
     * @param angle The desired [frc.robot.subsystems.hood.Hood] angle.
     */
    private fun setAngle(angle: Angle) {
        targetAngle = HoodConstants.PhysicalLimits.Limits.coerceIn(angle) as Angle
        motorController.positionRequestSubsystem(angle,
            HoodConstants.PhysicalLimits.Limits,
            HoodConstants.PhysicalLimits.Reduction)
    }

    /**
     * Uses the interpolation object to get the suitable [Angle] of the hood for the FUELS to
     * reach the HUB based on the [frc.robot.subsystems.shooter.Shooter] current velocity.
     * This [Angle] is then passed to [setAngle]
     * @param shooterRPMs The current velocity of the [frc.robot.subsystems.shooter.Shooter].
     */
    private fun setVelocityDrivenInterpolatedAngle(shooterRPMs: AngularVelocity) {
        // TODO() = Implement Interpolation
    }

    /**
     * Uses the interpolation object to get the suitable [Angle] of the hood for the FUELS to
     * reach the HUB based on the [frc.robot.subsystems.drivetrain.Drive] current distance to the HUB.
     * This [Angle] is then passed to [setAngle]
     * @param chassisDistanceToHUB The current distance from Swerve to HUB.
     */
    private fun setDistanceDrivenInterpolatedAngle(chassisDistanceToHUB: Distance) {
        // TODO() = Implement Interpolation
    }

    /**
     * Creates a voltage request to the motor controller. Used within the [SysIdSubsystem] interface to
     * run the characterization methods. [sysIdSetVoltage]
     */
    override fun setVoltage(voltage: Voltage) {
        motorController.voltageRequest(voltage)
    }

    // -------------------------------
    // PUBLIC — CMD Motors Control
    // -------------------------------

    /**
     * Command version of [setAngle]. Subsystem set as requirement.
     * @param angle The desired [frc.robot.subsystems.hood.Hood] angle.
     * @return an [InstantCommand] calling [setAngle]
     */
    fun setAngleCMD(angle: Angle): Command {
        return InstantCommand({ setAngle(angle)}, this)
    }

    /**
     * Command version of [setVelocityDrivenInterpolatedAngle]. Subsystem set as requirement.
     * @param shooterRPMs The current velocity of the [frc.robot.subsystems.shooter.Shooter].
     * @return a [RunCommand] calling [setVelocityDrivenInterpolatedAngle]
     */
    fun setVelocityDrivenInterpolatedAngleCMD(shooterRPMs: Supplier<AngularVelocity>): Command {
        return RunCommand({ setVelocityDrivenInterpolatedAngle(shooterRPMs.get()) }, this)
    }

    /**
     * Command version of [setDistanceDrivenInterpolatedAngle]. Subsystem set as requirement.
     * @param chassisDistanceToHUB The current distance from Swerve to HUB.
     * @return a [RunCommand] calling [setDistanceDrivenInterpolatedAngle]
     */
    fun setDistanceDrivenInterpolatedAngleCMD(chassisDistanceToHUB: Supplier<Distance>): Command {
        return RunCommand({ setDistanceDrivenInterpolatedAngle(chassisDistanceToHUB.get()) }, this)
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

    // -------------------------------
    // PUBLIC — Telemetry methods
    // -------------------------------

    /**
     * Returns the [frc.robot.subsystems.hood.Hood] angle by transforming the lead motor
     * reported position with the respective reduction.
     * This can be seen live in the "Hood" tab of AdvantageScope.
     */
    @AutoLogOutput(key = HoodConstants.Telemetry.HOOD_ANGLE_FIELD)
    fun getHoodAngle(): Angle {
        return HoodConstants.PhysicalLimits.Reduction.apply(
            motorController.position.value
        )
    }

    /**
     * Returns the [frc.robot.subsystems.hood.Hood] target angle.
     * This can be seen live in the "Hood" tab of AdvantageScope.
     */
    @AutoLogOutput(key = HoodConstants.Telemetry.HOOD_TARGET_ANGLE_FIELD)
    fun getHoodTargetAngle(): Angle {
        return targetAngle
    }

    // -------------------------------
    // PUBLIC — Neutral Mode control
    // -------------------------------

    /**
     * Sets the motor [com.ctre.phoenix6.signals.NeutralModeValue] to coast. Subsystem set as requirement.
     * @return an [InstantCommand] requesting motor coast.
     */
    fun coastCMD(): Command {
        return InstantCommand({ motorController.coast() }, this)
    }

    /**
     * Sets the motor [com.ctre.phoenix6.signals.NeutralModeValue] to brake. Subsystem set as requirement.
     * @return an [InstantCommand] requesting motor brake.
     */
    fun brakeCMD(): Command {
        return InstantCommand({ motorController.brake() }, this)
    }

    /**
     * Used to update the PIDF of the hood motor. Initial configuration remains the same, only [Slot0Configs] regarding
     * kP, kI, kD and kF are changed depending on the value passed to the [frc.robot.utils.controlProfiles.LoggedTunableNumber]
     * @param kP P coefficient received live
     * @param kI I coefficient received live
     * @param kD D coefficient received live
     * @param kF F coefficient received live
     */
    private fun updateHoodMotorPIDF(kP: Double, kI: Double, kD: Double, kF: Double) {
        val newSlot0Configs: Slot0Configs = KrakenMotors.configureSlot0(
            ControlGains(
                kP, kI, kD, kF,
                HoodConstants.Configuration.controlGains.s,
                HoodConstants.Configuration.controlGains.v,
                HoodConstants.Configuration.controlGains.a,
                HoodConstants.Configuration.controlGains.g
            )
        )

        motorController.applyConfigAndClearFaults(HoodConstants.Configuration.motorConfig.withSlot0(newSlot0Configs))
    }
}