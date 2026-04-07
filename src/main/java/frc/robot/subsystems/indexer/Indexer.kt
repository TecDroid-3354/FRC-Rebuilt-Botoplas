package frc.robot.subsystems.indexer

import com.ctre.phoenix6.configs.Slot0Configs
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.rotationsPerSecond
import frc.template.utils.volts
import org.littletonrobotics.junction.AutoLogOutput
import java.util.function.Supplier

class Indexer() : SubsystemBase() {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val hopperRollersMotor                  : OpTalonFX =
        OpTalonFX(IndexerConstants.Identification.HOPPER_ROLLERS_ID)
    
    private val leadTowerRollersMotor               : OpTalonFX =
        OpTalonFX(IndexerConstants.Identification.LEAD_TOWER_ROLLERS_ID)
    private val towerFollowerRollersMotor           : OpTalonFX =
        OpTalonFX(IndexerConstants.Identification.FOLLOWER_TOWER_ROLLERS_ID)

    // -------------------------------
    // PRIVATE — Enabled flags
    // -------------------------------
    private var hopperEnabled = false
    private var towerEnabled   = false

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val indexerRollersAlert : Alert =
        Alert(IndexerConstants.Telemetry.INDEXER_CONNECTED_ALERTS_FIELD,
            "Indexer Belts Motor ID ${IndexerConstants.Identification.HOPPER_ROLLERS_ID} Disconnected",
            Alert.AlertType.kError)

    private val leadTowerRollersAlert  : Alert =
        Alert(IndexerConstants.Telemetry.INDEXER_CONNECTED_ALERTS_FIELD,
            "Tower Rollers Motor ID ${IndexerConstants.Identification.LEAD_TOWER_ROLLERS_ID} Disconnected",
            Alert.AlertType.kError)

    private val followerTowerRollersAlert  : Alert =
        Alert(IndexerConstants.Telemetry.INDEXER_CONNECTED_ALERTS_FIELD,
            "Tower Rollers Motor ID ${IndexerConstants.Identification.FOLLOWER_TOWER_ROLLERS_ID} Disconnected",
            Alert.AlertType.kError)

    /**
     * Called upon [frc.robot.subsystems.indexer.Indexer] creation. Used to configure motors.
     */
    init {
        hopperRollersMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.hopperRollersConfig)
        leadTowerRollersMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.towerRollersConfig)
        towerFollowerRollersMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.towerRollersConfig)
        towerFollowerRollersMotor.follow(leadTowerRollersMotor.getMotorInstance(), IndexerConstants.Configuration.followerTowerMotorAlignmentValue)
    }

    /**
     * Called every 20ms loop. Used to update alerts and keep track of changes in voltage target values.
     */
    override fun periodic() {
        indexerRollersAlert.set(hopperRollersMotor.getIsConnected().not())
        leadTowerRollersAlert.set(leadTowerRollersMotor.getIsConnected().not())
        followerTowerRollersAlert.set(towerFollowerRollersMotor.getIsConnected().not())

        if (IndexerConstants.Tunables.HopperRollersVelocity.hasChanged(hashCode())) {
            updateHopperBeltsTargetVelocity(
                IndexerConstants.Tunables.HopperRollersVelocity.get()
            )
        }

        if (IndexerConstants.Tunables.towerRollersVelocity.hasChanged(hashCode())) {
            updateTowerRollersTargetVelocity(
                IndexerConstants.Tunables.towerRollersVelocity.get()
            )
        }

        if (IndexerConstants.Tunables.motorkP.hasChanged(hashCode())
            || IndexerConstants.Tunables.motorkI.hasChanged(hashCode())
            || IndexerConstants.Tunables.motorkD.hasChanged(hashCode())
            || IndexerConstants.Tunables.motorkF.hasChanged(hashCode())) {
            updateMotorsPIDF(
                IndexerConstants.Tunables.motorkP.get(), IndexerConstants.Tunables.motorkI.get(),
                IndexerConstants.Tunables.motorkD.get(), IndexerConstants.Tunables.motorkF.get()
            )
        }
    }

    // --------------------------------
    // PRIVATE — Component wise control
    // --------------------------------

    /**
     * Enables the hopper belts with the voltage stored in [IndexerConstants.RPSTargets].
     * IndexerRollersVoltage within the constants is used.
     */
    private fun enableHopperBelts(velocity: Supplier<AngularVelocity>) {
        hopperRollersMotor.velocityRequest(velocity.get())
        hopperEnabled = true
    }

    /**
     * Stops hopper belts.
     */
    private fun stopHopperBelts() {
        hopperRollersMotor.stopMotor()
        hopperEnabled = false
    }

    /**
     * Enables the tower rollers with the voltage stored in [IndexerConstants.RPSTargets].
     * TowerRollersVoltage within the constants is used.
     */
    private fun enableTowerRollers(velocity: Supplier<AngularVelocity>) {
        //leadTowerRollersMotor.velocityRequest(30.0.rotationsPerSecond)
        leadTowerRollersMotor.velocityRequest(velocity.get())
        towerEnabled   = true
    }

    /**
     * Stops tower rollers
     */
    private fun stopTowerRollers() {
        leadTowerRollersMotor.stopMotor()
        towerEnabled   = false
    }

    /**
     * Enables the hopper belts with the voltage stored in [IndexerConstants.RPSTargets].
     * IndexerRollersVoltage within the constants is used.
     */
    private fun enableHopperBeltsReversed() {
        hopperRollersMotor.velocityRequest(-IndexerConstants.RPSTargets.HopperRollersVelocity)
        hopperEnabled = true
    }

    /**
     * Enables the tower rollers with the voltage stored in [IndexerConstants.RPSTargets].
     * TowerRollersVoltage within the constants is used.
     */
    private fun enableTowerRollersReversed() {
        leadTowerRollersMotor.velocityRequest(-IndexerConstants.RPSTargets.TowerRollersVelocity)
        towerEnabled   = true
    }

    // ----------------------------------------
    // PUBLIC — CMD component wise control
    // ----------------------------------------

    /**
     * Command version of [enableHopperBelts].
     */
    fun enableHopperBeltsCMD(): Command {
        return InstantCommand({ enableHopperBelts { IndexerConstants.RPSTargets.HopperRollersVelocity } })
    }

    fun enableHopperBeltsIdleCMD(): Command {
        return InstantCommand({ enableHopperBelts { IndexerConstants.RPSTargets.HopperRollersIdleVelocity } })
    }

    /**
     * Command version of [stopHopperBelts].
     */
    private fun stopHopperBeltsCMD(): Command {
        return InstantCommand({ stopHopperBelts() })
    }

    /**
     * Command version of [enableTowerRollers].
     */
    fun enableTowerRollersCMD(): Command {
        return InstantCommand({ enableTowerRollers { IndexerConstants.RPSTargets.TowerRollersVelocity } })
    }

    fun enableTowerRollersIdleCMD(): Command {
        return InstantCommand({ enableTowerRollers { IndexerConstants.RPSTargets.TowerRollersIdleVelocity } })
    }

    /**
     * Command version of [stopTowerRollers].
     */
    private fun stopTowerRollersCMD(): Command {
        return InstantCommand({ stopTowerRollers() })
    }

    // ----------------------------------------
    // PUBLIC — CMD subsystem wise control
    // ----------------------------------------

    /**
     * Wrapper command of [enableTowerRollersCMD] and [enableHopperBeltsCMD].
     * @return
     */
    fun enableIndexerCMD(): Command {
        return ParallelCommandGroup(
            enableHopperBeltsCMD(),
            enableTowerRollersCMD()
        )
    }

    fun enableIndexerIdleCMD(): Command {
        return ParallelCommandGroup(
            enableHopperBeltsIdleCMD(),
            enableTowerRollersIdleCMD()
        )
    }

    /**
     * Wrapper command of [stopHopperBeltsCMD] and [stopTowerRollersCMD]
     */
    fun stopIndexerCMD(): Command {
        return ParallelCommandGroup(
            stopHopperBeltsCMD(),
            stopTowerRollersCMD()
        )
    }

    // -------------------------------
    // PUBLIC — Telemetry methods
    // -------------------------------


    @AutoLogOutput
    fun getTowerVelocity(): AngularVelocity {
        return IndexerConstants.PhysicalLimits.TowerReduction.apply(leadTowerRollersMotor.getVelocity())
    }

    @AutoLogOutput
    fun getHopperVelocity(): AngularVelocity {
        return IndexerConstants.PhysicalLimits.HopperReduction.apply(hopperRollersMotor.getVelocity())
    }

    @AutoLogOutput
    fun getTargetTowerVelocity(): AngularVelocity {
        return IndexerConstants.RPSTargets.TowerRollersVelocity
    }

    @AutoLogOutput
    fun getTargetHopperVelocity(): AngularVelocity {
        return IndexerConstants.RPSTargets.HopperRollersVelocity
    }

    /**
     * Returns whether the indexer components are enabled.
     * This can be seen live in the "Indexer" tab of AdvantageScope.
     */
    @AutoLogOutput(key = IndexerConstants.Telemetry.HOPPER_ENABLED_FIELD)
    fun isIndexerEnabled(): Boolean {
        return hopperEnabled
    }

    /**
     * Returns whether the tower components are enabled.
     * This can be seen live in the "Indexer" tab of AdvantageScope.
     */
    @AutoLogOutput(key = IndexerConstants.Telemetry.TOWER_ENABLED_FIELD)
    fun isTowerEnabled(): Boolean {
        return towerEnabled
    }

    /**
     * Used to update the velocity targets of the rollers component.
     * @param beltsVelocity Velocity target for belts. Received live
     */
    private fun updateHopperBeltsTargetVelocity(beltsVelocity: Double) {
        IndexerConstants.RPSTargets.HopperRollersVelocity = beltsVelocity.rotationsPerSecond
    }

    /**
     * Used to update the velocity targets of the rollers component.
     * @param rollersVelocity Velocity target for tower rollers. Received live
     */
    private fun updateTowerRollersTargetVelocity(rollersVelocity: Double) {
        IndexerConstants.RPSTargets.TowerRollersVelocity = rollersVelocity.rotationsPerSecond
    }

    /**
     * Used to update the PIDF of all motors. Initial configuration remains the same, only [Slot0Configs] regarding
     * kP, kI, kD and kF are changed depending on the value passed to the [frc.robot.utils.controlProfiles.LoggedTunableNumber]
     * @param kP P coefficient received live
     * @param kI I coefficient received live
     * @param kD D coefficient received live
     * @param kF F coefficient received live
     */
    private fun updateMotorsPIDF(kP: Double, kI: Double, kD: Double, kF: Double) {
        val newSlot0Configs: Slot0Configs = KrakenMotors.configureSlot0(
            ControlGains(kP, kI, kD, kF,
                IndexerConstants.Configuration.controlGains.s,
                IndexerConstants.Configuration.controlGains.v,
                IndexerConstants.Configuration.controlGains.a,
                IndexerConstants.Configuration.controlGains.g)
        )

        hopperRollersMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.hopperRollersConfig.withSlot0(newSlot0Configs))
        leadTowerRollersMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.towerRollersConfig.withSlot0(newSlot0Configs))
        towerFollowerRollersMotor.applyConfigAndClearFaults(IndexerConstants.Configuration.towerRollersConfig.withSlot0(newSlot0Configs))
    }
}
