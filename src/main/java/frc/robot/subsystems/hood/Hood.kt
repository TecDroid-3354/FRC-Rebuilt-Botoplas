package frc.robot.subsystems.hood
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.utils.subsystemUtils.generic.SysIdSubsystem
import frc.robot.utils.subsystemUtils.identification.SysIdRoutines
import frc.template.utils.devices.OpTalonFX
import org.littletonrobotics.junction.AutoLogOutput

class Hood() : SysIdSubsystem("Hood") {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val motorController     : OpTalonFX =
        OpTalonFX(HoodConstants.Identification.HOOD_MOTOR_ID)

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var targetPose          : Angle = Degrees.zero()

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
     * Called every 20ms loop. Used to update alerts.
     * TODO() = Update motor's PID through here.
     */
    override fun periodic() {
        hoodConnectedAlert.set(motorController.isConnected.invoke().not())
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
        targetPose = HoodConstants.PhysicalLimits.Limits.coerceIn(angle) as Angle
        motorController.positionRequestSubsystem(angle,
            HoodConstants.PhysicalLimits.Limits,
            HoodConstants.PhysicalLimits.Reduction)
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
        return InstantCommand({ setAngle(angle)},this)
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
        return targetPose
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
}