package frc.robot.subsystems.hood
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.subsystems.indexer.IndexerConstants
import frc.template.utils.degrees
import frc.template.utils.devices.OpTalonFX
import org.littletonrobotics.junction.AutoLogOutput

class Hood() : Subsystem {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val motorController     : OpTalonFX =
        OpTalonFX(HoodConstants.Identification.HOOD_MOTOR_ID)

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var targetPose          : Angle = 0.0.degrees

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val hoodConnectedAlert: Alert =
        Alert(HoodConstants.Telemetry.HOOD_CONNECTED_ALERTS_FIELD,
            "Hood Motor ID ${HoodConstants.Identification.HOOD_MOTOR_ID} Disconnected",
            Alert.AlertType.kError)

    /**
     * Called upon [frc.robot.subsystems.hood.Hood] creation. Used to configure motors.
     */
    init {
        motorController.applyConfigAndClearFaults(HoodConstants.Configuration.motorConfig)
    }

    /**
     * Called every 20ms loop. Used to update alerts.
     * TODO() = Update deployable component motors' PID through here.
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