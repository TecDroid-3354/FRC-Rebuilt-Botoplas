package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Subsystem

class Climber() : Subsystem {

    private val leader = TalonFX(ClimberConstants.Identification.LEADER_MOTOR_ID)
    private val follower = TalonFX(ClimberConstants.Identification.FOLLOWER_MOTOR_ID)

    private val absolute: CANcoder = CANcoder(ClimberConstants.Identification.ABSOLUTE_ID)

    private val positionRequest = PositionVoltage(0.0)

    init {
        //Set "follower" motor to mimic "leader" motor
        follower.setControl(Follower(leader.deviceID, MotorAlignmentValue.Opposed))

        //Match relative encoder with absolute encoder
        syncRelativeToAbsolute()
    }

    /**
     * Moves the climber to the climb position
     */
    fun climb() {
        val desiredPosition = ClimberConstants.PhysicalLimits.IDLE_POSITION

        //val clampedPosition = ClimberConstants.PhysicalLimits.LIMITS_clamp()

        val motorPosition = desiredPosition * ClimberConstants.PhysicalLimits.GEAR_RATIO

        leader.setControl(positionRequest.withPosition(motorPosition))

    }

    /**
     * Returns the climber to its idle (starting) position
     */
    fun idleClimb() {
        val desiredPosition = ClimberConstants.PhysicalLimits.IDLE_POSITION

        //val clampedPosition = ClimberConstants.PhysicalLimits.LIMITS_clamp()

        val motorPosition = desiredPosition * ClimberConstants.PhysicalLimits.GEAR_RATIO

        leader.setControl(positionRequest.withPosition(motorPosition))

    }

    fun getPosition(): Angle {
        return leader.position.value * ClimberConstants.PhysicalLimits.GEAR_RATIO

    }

    /**
     * Syncs the motors relative encoder to the absolute encoder
     * Required for correct limit clamping
     */
    private fun syncRelativeToAbsolute() {
        leader.setPosition(absolute.position.value.times(ClimberConstants.PhysicalLimits.GEAR_RATIO))
    }
}