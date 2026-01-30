package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem

import frc.template.utils.devices.OpTalonFX

class Shooter() : Subsystem {

    val motionMagicRequest = MotionMagicVelocityVoltage(0.0)

    val motorController = OpTalonFX(ShooterConstants.Identification.LeadMotorID)

    val motorFollowerOne = OpTalonFX(ShooterConstants.Identification.MotorFollowerOneID)
    val motorFollowerTwo = OpTalonFX(ShooterConstants.Identification.MotorFollowerTwoID)
    val motorFollowerThree = OpTalonFX(ShooterConstants.Identification.MotorFollowerThreeID)

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

    fun setVelocity(velocity : AngularVelocity){
        motorController.getMotorInstance().setControl(motionMagicRequest.withVelocity(velocity))
    }

    fun stopShooter(){
        motorController.getMotorInstance().setControl(motionMagicRequest.withVelocity(0.0))
    }

    fun setVelocityCMD(speed : AngularVelocity){
        val clampedVelocity = speed.coerceIn(RotationsPerSecond.of(-6_000.0)..RotationsPerSecond.of(6_000.0))
        InstantCommand({  setVelocity(clampedVelocity) }, this)
    }

    fun stopShooterCMD(){
        InstantCommand({ stopShooter() }, this)
    }

}