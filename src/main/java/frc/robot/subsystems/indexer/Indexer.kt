package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.RobotConstants
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.volts
import org.littletonrobotics.junction.AutoLogOutput

class Indexer() : SubsystemBase() {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val indexerRollersMotor: OpTalonFX =
        OpTalonFX(IndexerConstants.Identification.INDEXER_ROLLERS_ID)

    // -------------------------------
    // PRIVATE — Enabled flag
    // -------------------------------
    private var indexerEnabled = false

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val indexerRollersAlert: Alert =
        Alert(IndexerConstants.Telemetry.INDEXER_CONNECTED_ALERTS_FIELD,
            "Indexer Rollers Motor ID ${IndexerConstants.Identification.INDEXER_ROLLERS_ID} Disconnected",
            Alert.AlertType.kError)

    /**
     * Called upon [frc.robot.subsystems.indexer.Indexer] creation. Used to configure motors.
     */
    init {
        indexerRollersMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.motorConfig)
    }

    /**
     * Called every 20ms loop. Used to update alerts and keep track of changes in voltage target values.
     */
    override fun periodic() {
        indexerRollersAlert.set(indexerRollersMotor.isConnected.invoke().not())

        if (IndexerConstants.Tunables.indexerRollersVoltage.hasChanged(hashCode())) {
            updateRollersTargetVoltage(
                IndexerConstants.Tunables.indexerRollersVoltage.get()
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
        indexerRollersMotor.voltageRequest(IndexerConstants.VoltageTargets.RollersVoltage)
        indexerEnabled = true
    }

    /**
     * Stops hopper (bottom) rollers.
     */
    private fun stopHopperRollers() {
        indexerRollersMotor.stopMotor()
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
        return InstantCommand({ enableHopperRollers() }, this)
    }

    /**
     * Command version of [stopIndexer]. Subsystem set as requirement.
     */
    fun stopIndexerCMD(): Command {
        return InstantCommand({ stopHopperRollers() }, this)
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
     * @param rollersVoltage Voltage target for bottom rollores. Received live
     * @param lateralRollerVoltage Voltage target for lateral rollers. Received live
     */
    private fun updateRollersTargetVoltage(rollersVoltage: Double) {
        IndexerConstants.VoltageTargets.RollersVoltage = rollersVoltage.volts
    }
}
