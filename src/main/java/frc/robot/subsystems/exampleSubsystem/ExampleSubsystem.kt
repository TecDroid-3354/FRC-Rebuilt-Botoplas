package frc.robot.subsystems.exampleSubsystem

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Subsystem

class ExampleSubsystem() : Subsystem {
    private val motorController = TalonFX(ExampleConstants.MOTOR_ID)

    init {
        configureMotorInterface()
    }

    fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()
        talonConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
        talonConfig.CurrentLimits
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ExampleConstants.MOTOR_CURRENT_LIMITS)


        motorController.clearStickyFaults()
        motorController.configurator.apply(talonConfig)
    }
}