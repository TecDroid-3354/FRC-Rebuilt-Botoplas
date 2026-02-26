package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.RobotConstants
import frc.robot.utils.interpolation.InterpolatingDouble
import frc.robot.utils.interpolation.InterpolatingTreeMap
import frc.robot.utils.subsystemUtils.generic.SysIdSubsystem
import frc.robot.utils.subsystemUtils.identification.SysIdRoutines
import frc.template.utils.controlProfiles.ControlGains
import frc.template.utils.degreesPerSecond
import frc.template.utils.devices.KrakenMotors

import frc.template.utils.devices.OpTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import java.util.function.Supplier

class Shooter() : SysIdSubsystem("Shooter") {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val leadMotorController     : OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.LEAD_MOTOR_LEFT_SHOOTER_FIRST_ID)

    private val followerLeftMotorSecond : OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_LEFT_SHOOTER_SECOND_ID)
    private val followerRightMotorFirst : OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_FIRST_ID)
    private val followerRightMotorSecond: OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_SECOND_ID)

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var targetVelocity          : AngularVelocity = DegreesPerSecond.zero()
    private val shooterInterpolation: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap()

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val leadLeftFirstAlert          : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Left Shooter First Motor ID ${ShooterConstants.Identification.LEAD_MOTOR_LEFT_SHOOTER_FIRST_ID} Disconnected",
            Alert.AlertType.kError)
    private val followerLeftSecondAlert     : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Left Shooter Second Motor ID ${ShooterConstants.Identification.FOLLOWER_LEFT_SHOOTER_SECOND_ID} Disconnected",
            Alert.AlertType.kError)
    private val followerRightFirstAlert     : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Right Shooter First Motor ID ${ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_FIRST_ID} Disconnected",
            Alert.AlertType.kError)
    private val followerRightSecondAlert    : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Right Shooter Second Motor ID ${ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_SECOND_ID} Disconnected",
            Alert.AlertType.kError)

    // --------------------------------------------------------------------------------
    // PRIVATE SYS ID — Running Conditions, Relevant Variables and Routines Declaration
    // --------------------------------------------------------------------------------
    override val sysIdForwardRunningCondition: () -> Boolean = { getShooterAngularVelocity() < ShooterConstants.Control.MAX_RPS }
    override val sysIdBackwardRunningCondition: () -> Boolean = { getShooterAngularVelocity() > ShooterConstants.Control.MIN_RPS }

    /**
     * [motorPosition] holds the current motor's position without gear ratios
     */
    override val motorPosition: Angle
        get() = leadMotorController.position.value
    /**
     * [motorVelocity] holds the current motor's velocity without gear ratios
     */
    override val motorVelocity: AngularVelocity
        get() = leadMotorController.velocity.value
    /**
     * [power] holds the current motor's power
     */
    override val power: Double
        get() = leadMotorController.power.invoke()

    /**
     * [sysIdRoutines] holds the 4 possible SysId routines, later called in [sysIdQuasistaticRoutine] & [sysIdDynamicRoutine]
     */
    private val sysIdRoutines: SysIdRoutines = createIdentificationRoutines().createTests()

    /**
     * Called upon [frc.robot.subsystems.shooter.Shooter] creation. Used to call motors config method.
     */
    init {
        motorConfiguration()
        interpolationConfiguration()
    }

    /**
     * Called every 20ms loop. Used to update alerts and keep track of changes in PIDF values.
     */
    override fun periodic() {
        leadLeftFirstAlert.set(leadMotorController.isConnected.invoke().not())
        followerLeftSecondAlert.set(followerLeftMotorSecond.isConnected.invoke().not())
        followerRightFirstAlert.set(followerRightMotorFirst.isConnected.invoke().not())
        followerRightSecondAlert.set(followerRightMotorSecond.isConnected.invoke().not())

        if (ShooterConstants.Tunables.motorkP.hasChanged(hashCode())
            || ShooterConstants.Tunables.motorkI.hasChanged(hashCode())
            || ShooterConstants.Tunables.motorkD.hasChanged(hashCode())
            || ShooterConstants.Tunables.motorkF.hasChanged(hashCode())) {
            updateMotorsPIDF(
                ShooterConstants.Tunables.motorkP.get(), ShooterConstants.Tunables.motorkI.get(),
                ShooterConstants.Tunables.motorkD.get(), ShooterConstants.Tunables.motorkF.get())
        }
    }

    // ---------------------------------
    // PRIVATE — Runnable Motors Control
    // ---------------------------------

    /**
     * Clamps the desired [AngularVelocity] between [[-100, 100]] rotations
     * per second, as that is the max for Kraken motors (6_000 RPMs). It is applied to all
     * [frc.robot.subsystems.shooter.Shooter] motors through a [MotionMagicVelocityVoltage] request.
     * @param velocity The desired MOTOR [AngularVelocity].
     */
    private fun setVelocity(velocity : AngularVelocity) {
        val clampedVelocity = velocity.coerceIn(ShooterConstants.Control.MIN_RPS..ShooterConstants.Control.MAX_RPS)

        targetVelocity = clampedVelocity
        leadMotorController.velocityRequest(clampedVelocity)
    }

    /**
     * Uses the interpolation object to get the suitable [AngularVelocity] of the shooter motors for the FUELS to
     * reach the target. This [AngularVelocity] is then passed to [setVelocity].
     * - NOTE: Interpolation is performed in [Meters] and [DegreesPerSecond], hence the units inside the method.
     * @param distanceToTarget The current distance from chassis to the target (HUB or passed the trench to assist).
     */
    private fun setInterpolatedVelocity(distanceToTarget: Distance) {
        val shooterSetpointDps = shooterInterpolation.getInterpolated(InterpolatingDouble(distanceToTarget.`in`(Meters)))
        setVelocity(shooterSetpointDps.value.degreesPerSecond)
    }

    /**
     * Stops all [frc.robot.subsystems.shooter.Shooter] motors.
     */
    private fun stopShooter() {
        targetVelocity = DegreesPerSecond.zero()
        leadMotorController.stopMotor()
    }

    override fun setVoltage(voltage: Voltage) {
        leadMotorController.voltageRequest(voltage)
    }

    // -------------------------------
    // PUBLIC — CMD Motors Control
    // -------------------------------

    /**
     * Command version of [setVelocity]. Subsystem set as requirement.
     * @param velocity The desired [AngularVelocity] of the MOTORS.
     * @return an [InstantCommand] calling [setVelocity]
     */
    fun setVelocityCMD(velocity : Supplier<AngularVelocity>): Command {
        return InstantCommand({ setVelocity(velocity.get()) }, this)
    }

    /**
     * Command version of [setInterpolatedVelocity]. Subsystem set as requirement.
     * @param distanceToTarget The current distance from chassis to the target (HUB or passed the trench to assist).
     * @return a [RunCommand] calling [setInterpolatedVelocity]
     */
    fun setInterpolatedVelocityCMD(distanceToTarget: Supplier<Distance>): Command {
        return RunCommand({ setInterpolatedVelocity(distanceToTarget.get()) }, this)
    }

    /**
     * Command version of [stopShooter]. Subsystem set as requirement.
     * @return an [InstantCommand] calling [stopShooter]
     */
    fun stopShooterCMD(): Command {
        return InstantCommand({ stopShooter() }, this)
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
     * Returns the [AngularVelocity] reported by the lead motor.
     * This can be seen live in the "Shooter" tab of AdvantageScope.
     */
    @AutoLogOutput(key = ShooterConstants.Telemetry.SHOOTER_RPM_FIELD)
    fun getShooterAngularVelocity(): AngularVelocity {
        return leadMotorController.velocity.value
    }

    /**
     * Returns the target [AngularVelocity].
     * This can be seen live in the "Shooter" tab of AdvantageScope.
     */
    @AutoLogOutput(key = ShooterConstants.Telemetry.SHOOTER_TARGET_RPM_FIELD)
    private fun getShooterTargetAngularVelocity(): AngularVelocity {
        return targetVelocity
    }

    // -------------------------------
    // Motor configuration (Phoenix 6)
    // -------------------------------

    /**
     * Puts every interpolation point in the interpolation object.
     * - key    : Distance to target in [Meters]
     * - value  : Shooter velocity in [DegreesPerSecond]
     */
    private fun interpolationConfiguration() {
        for (point in ShooterConstants.Control.shooterInterpolationPoints) {
            shooterInterpolation.put(
                InterpolatingDouble(point.key.`in`(Meters)),
                InterpolatingDouble(point.value.`in`(DegreesPerSecond)))
        }
    }

    /**
     * Apply the config defined in [ShooterConstants.Configuration] to each motor.
     * Follower requests are also applied.
     */
    private fun motorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)

        followerLeftMotorSecond.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        followerRightMotorFirst.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        followerRightMotorSecond.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)

        followerLeftMotorSecond.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Aligned)
        followerRightMotorFirst.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Opposed)
        followerRightMotorSecond.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Opposed)
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
                ShooterConstants.Configuration.controlGains.s,
                ShooterConstants.Configuration.controlGains.v,
                ShooterConstants.Configuration.controlGains.a,
                ShooterConstants.Configuration.controlGains.g)
        )

        leadMotorController.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig.withSlot0(newSlot0Configs))
        followerLeftMotorSecond.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig.withSlot0(newSlot0Configs))
        followerRightMotorFirst.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig.withSlot0(newSlot0Configs))
        followerRightMotorSecond.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig.withSlot0(newSlot0Configs))
    }
}