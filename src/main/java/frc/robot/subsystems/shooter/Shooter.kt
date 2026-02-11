package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem

import frc.template.utils.devices.OpTalonFX

class Shooter() : Subsystem {

    private val motionMagicRequest = MotionMagicVelocityVoltage(0.0)

    private val motorController = OpTalonFX(ShooterConstants.Identification.LeadMotorID)

    private val motorFollowerOne = OpTalonFX(ShooterConstants.Identification.MotorFollowerOneID)
    private val motorFollowerTwo = OpTalonFX(ShooterConstants.Identification.MotorFollowerTwoID)
    private val motorFollowerThree = OpTalonFX(ShooterConstants.Identification.MotorFollowerThreeID)

    init{
        motorConfiguration()
    }

    fun motorConfiguration(){
        motorController.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)

        motorFollowerOne.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        motorFollowerTwo.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)
        motorFollowerThree.applyConfigAndClearFaults(ShooterConstants.Configuration.motorsConfig)

        motorFollowerOne.follow(motorController.getMotorInstance(), MotorAlignmentValue.Aligned)
        motorFollowerTwo.follow(motorController.getMotorInstance(), MotorAlignmentValue.Aligned)
        motorFollowerThree.follow(motorController.getMotorInstance(), MotorAlignmentValue.Aligned)





    }

    private fun setVelocity(velocity : AngularVelocity){
        motorController.getMotorInstance().setControl(motionMagicRequest.withVelocity(velocity))
    }

    private fun stopShooter(){
         motorController.getMotorInstance().setControl(motionMagicRequest.withVelocity(0.0))
    }

    fun setVelocityCMD(speed : AngularVelocity) : Command {
        val clampedVelocity = speed.coerceIn(RotationsPerSecond.of(-100.0)..RotationsPerSecond.of(100.0))
        return InstantCommand({  setVelocity(clampedVelocity) }, this)
    }

    fun stopShooterCMD(): Command {
        return InstantCommand({ stopShooter() }, this)
    }

}