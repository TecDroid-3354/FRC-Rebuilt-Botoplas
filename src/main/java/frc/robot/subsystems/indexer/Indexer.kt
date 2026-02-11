package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.template.utils.devices.OpTalonFX

class Indexer : Subsystem {
    //Identification of the motors
    private val bottomRollerMotor =
        OpTalonFX(IndexerConstants.Identification.BottomRollerMotorID)

    private var indexerEnabled = false

    init {
        motorConfiguration()
    }
    //Clear faults of the Motors
    private fun motorConfiguration() {
        bottomRollerMotor.applyConfigAndClearFaults(
            IndexerConstants.Configuration.motorConfig
        )
    }

    /* ---------------- Private motor actions ---------------- */
    //Apply Indexer constant to enable the roller's voltage
    private fun enableHopperRollers() {
        bottomRollerMotor.getMotorInstance()
            .setVoltage(IndexerConstants.Voltage.BottomRollerVoltage)
    }
    //Apply Indexer constants to disable the roller's voltage
    private fun stopHopperRollers() {
        bottomRollerMotor.getMotorInstance().setVoltage(0.0)
    }


    /* ---------------- Public control methods ---------------- */

    fun enableIndexer() {
        enableHopperRollers()
        indexerEnabled = true
    }

    fun stopIndexer() {
        stopHopperRollers()
        indexerEnabled = false
    }

    fun isIndexerEnabled(): Boolean {
        return indexerEnabled
    }
}
