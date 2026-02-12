package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.template.utils.degrees
import frc.template.utils.degreesPerSecond

import frc.template.utils.devices.OpTalonFX
import org.littletonrobotics.junction.AutoLogOutput

class Shooter() : Subsystem {
    // -------------------------------
    // PRIVATE — Motors Declaration
    // -------------------------------
    private val leadMotorController     : OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.LEAD_MOTOR_LEFT_SHOOTER_FIRST_ID)

    private val followerLeftMotorSecond : OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_LEFT_SHOOTER_SECOND_ID)
    private val followerLeftMotorThird : OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_LEFT_SHOOTER_THIRD_ID)
    private val followerRightMotorFirst : OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_FIRST_ID)
    private val followerRightMotorSecond: OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_SECOND_ID)
    private val followerRightMotorThird: OpTalonFX =
        OpTalonFX(ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_THIRD_ID)

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var targetVelocity          : AngularVelocity = DegreesPerSecond.zero()

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
    private val followerLeftThirdAlert      : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Left Shooter Third Motor ID ${ShooterConstants.Identification.FOLLOWER_LEFT_SHOOTER_THIRD_ID} Disconnected",
            Alert.AlertType.kError)
    private val followerRightFirstAlert     : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Right Shooter First Motor ID ${ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_FIRST_ID} Disconnected",
            Alert.AlertType.kError)
    private val followerRightSecondAlert    : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Right Shooter Second Motor ID ${ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_SECOND_ID} Disconnected",
            Alert.AlertType.kError)
    private val followerRightThirdAlert     : Alert =
        Alert(ShooterConstants.Telemetry.SHOOTER_CONNECTED_ALERTS_FIELD,
            "Right Shooter Third Motor ID ${ShooterConstants.Identification.FOLLOWER_RIGHT_SHOOTER_THIRD_ID} Disconnected",
            Alert.AlertType.kError)

    /**
     * Called upon [frc.robot.subsystems.shooter.Shooter] creation. Used to call motors config method.
     */
    init {
        motorConfiguration()
    }

    /**
     * Called every 20ms loop. Used to update alerts.
     * TODO() = Update deployable component motors' PID through here.
     */
    override fun periodic() {
        leadLeftFirstAlert.set(leadMotorController.isConnected.invoke().not())
        followerLeftSecondAlert.set(followerLeftMotorSecond.isConnected.invoke().not())
        followerLeftThirdAlert.set(followerLeftMotorThird.isConnected.invoke().not())
        followerRightFirstAlert.set(followerRightMotorFirst.isConnected.invoke().not())
        followerRightSecondAlert.set(followerRightMotorSecond.isConnected.invoke().not())
        followerRightThirdAlert.set(followerRightMotorThird.isConnected.invoke().not())
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
     * Stops all [frc.robot.subsystems.shooter.Shooter] motors.
     */
    private fun stopShooter() {
        targetVelocity = DegreesPerSecond.zero()
        leadMotorController.stopMotor()
    }

    // -------------------------------
    // PUBLIC — CMD Motors Control
    // -------------------------------

    /**
     * Command version of [setVelocity]. Subsystem set as requirement.
     * @param velocity The desired [AngularVelocity] of the MOTORS.
     * @return an [InstantCommand] calling [setVelocity]
     */
    fun setVelocityCMD(velocity : AngularVelocity): Command {
        return InstantCommand({ setVelocity(velocity) }, this)
    }

    /**
     * Command version of [stopShooter]. Subsystem set as requirement.
     * @return an [InstantCommand] calling [stopShooter]
     */
    fun stopShooterCMD(): Command {
        return InstantCommand({ stopShooter() }, this)
    }

    // -------------------------------
    // PUBLIC — Telemetry methods
    // -------------------------------

    /**
     * Returns the [AngularVelocity] reported by the lead motor.
     * This can be seen live in the "Shooter" tab of AdvantageScope.
     */
    @AutoLogOutput(key = ShooterConstants.Telemetry.SHOOTER_RPM_FIELD)
    private fun getShooterAngularVelocity(): AngularVelocity {
        return leadMotorController.velocity.value
    }

    /**
     * Returns the [AngularVelocity] reported by the lead motor.
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
     * Apply the config defined in [ShooterConstants.Configuration] to each motor.
     * Follower requests are also applied.
     */
    private fun motorConfiguration() {
        leadMotorController.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)

        followerLeftMotorSecond.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        followerLeftMotorThird.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        followerRightMotorFirst.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        followerRightMotorSecond.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        followerRightMotorThird.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)

        followerLeftMotorSecond.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Aligned)
        followerLeftMotorThird.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Aligned)
        followerRightMotorFirst.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Aligned)
        followerRightMotorSecond.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Aligned)
        followerRightMotorThird.follow(leadMotorController.getMotorInstance(), MotorAlignmentValue.Aligned)
    }
}