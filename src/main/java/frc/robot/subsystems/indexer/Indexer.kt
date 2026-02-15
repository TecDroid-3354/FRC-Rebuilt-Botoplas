package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.shooter.IntakeConstants
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.volts
import org.littletonrobotics.junction.AutoLogOutput

class Indexer() : SubsystemBase() {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val bottomRollerMotor: OpTalonFX =
        OpTalonFX(IndexerConstants.Identification.BOTTOM_ROLLERS_ID)

    private val lateralRollerMotor: OpTalonFX =
        OpTalonFX(IndexerConstants.Identification.LATERAL_ROLLERS_ID)

    // -------------------------------
    // PRIVATE — Enabled flag
    // -------------------------------
    private var indexerEnabled = false

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val bottomRollersAlert: Alert =
        Alert(IndexerConstants.Telemetry.INDEXER_CONNECTED_ALERTS_FIELD,
            "Indexer Bottom Rollers Motor ID ${IndexerConstants.Identification.BOTTOM_ROLLERS_ID} Disconnected",
            Alert.AlertType.kError)
    private val lateralRollersAlert: Alert =
        Alert(IndexerConstants.Telemetry.INDEXER_CONNECTED_ALERTS_FIELD,
            "Indexer Lateral Rollers Motor ID ${IndexerConstants.Identification.LATERAL_ROLLERS_ID} Disconnected",
            Alert.AlertType.kError)

    /**
     * Called upon [frc.robot.subsystems.indexer.Indexer] creation. Used to configure motors.
     */
    init {
        bottomRollerMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.motorConfig)
        lateralRollerMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.motorConfig)
    }

    /**
     * Called every 20ms loop. Used to update alerts.
     * TODO() = Update deployable component motors' PID through here.
     */
    override fun periodic() {
        bottomRollersAlert.set(bottomRollerMotor.isConnected.invoke().not())
        lateralRollersAlert.set(lateralRollerMotor.isConnected.invoke().not())

        if (IndexerConstants.Tunables.bottomRollerVoltage.hasChanged(hashCode())
            || IndexerConstants.Tunables.lateralRollerVoltage.hasChanged(hashCode())) {
            updateRollersTargetVoltage(
                IndexerConstants.Tunables.bottomRollerVoltage.get(),
                IndexerConstants.Tunables.lateralRollerVoltage.get()
            )
        }
    }

    // --------------------------------
    // PRIVATE — Component wise control
    // --------------------------------

    /**
     * Enables the hopper rollers with the voltage stored in [IndexerConstants.VoltageTargets].
     * BottomRollerVoltage within the constants is used.
     */
    private fun enableHopperRollers() {
        bottomRollerMotor.voltageRequest(IndexerConstants.VoltageTargets.BottomRollerVoltage)
    }

    /**
     * Enables the hopper to shooter rollers with the voltage stored in [IndexerConstants.VoltageTargets].
     * LateralRollerVoltage within the constants is used.
     */
    private fun enableHopperToShooterRollers() {
        lateralRollerMotor.voltageRequest(IndexerConstants.VoltageTargets.LateralRollerVoltage)
    }

    /**
     * Stops hopper (bottom) rollers.
     */
    private fun stopHopperRollers() {
        bottomRollerMotor.stopMotor()
    }

    /**
     * Stops hopper (lateral) rollers.
     */
    private fun stopHopperToShooterRollers() {
        lateralRollerMotor.stopMotor()
    }

    // ----------------------------------------
    // PUBLIC — Runnable subsystem wise control
    // ----------------------------------------

    /**
     * Enables each indexer component. sets [indexerEnabled] to true.
     */
    private fun enableIndexer() {
        enableHopperRollers()
        enableHopperToShooterRollers()
        indexerEnabled = true
    }

    /**
     * Disables each indexer component. sets [indexerEnabled] to false.
     */
    private fun stopIndexer() {
        stopHopperRollers()
        stopHopperToShooterRollers()
        indexerEnabled = false
    }

    // ----------------------------------------
    // PUBLIC — CMD subsystem wise control
    // ----------------------------------------

    /**
     * Command version of [enableIndexer]. Subsystem set as requirement.
     * @return
     */
    fun enableIndexerCMD(): Command {
        return InstantCommand({ enableIndexer() }, this)
    }

    /**
     * Command version of [stopIndexer]. Subsystem set as requirement.
     */
    fun stopIndexerCMD(): Command {
        return InstantCommand({ stopIndexer() }, this)
    }

    // -------------------------------
    // PUBLIC — Telemetry methods
    // -------------------------------

    /**
     * Returns whether the indexer components are enabled.
     * This can be seen live in the "Indexer" tab of AdvantageScope.
     */
    @AutoLogOutput(key = IndexerConstants.Telemetry.INDEXER_ENABLED_FIELD)
    fun isIndexerEnabled(): Boolean {
        return indexerEnabled
    }
    /**
     * Used to update the voltage targets of the rollers component.
     * @param bottomRollerVoltage Voltage target for bottom rollores. Received live
     * @param lateralRollerVoltage Voltage target for lateral rollers. Received live
     */
    private fun updateRollersTargetVoltage(bottomRollerVoltage: Double, lateralRollerVoltage: Double) {
        IndexerConstants.VoltageTargets.BottomRollerVoltage = bottomRollerVoltage.volts
        IndexerConstants.VoltageTargets.LateralRollerVoltage = lateralRollerVoltage.volts
    }
    
}
