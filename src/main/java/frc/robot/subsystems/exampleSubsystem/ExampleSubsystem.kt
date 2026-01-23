package frc.robot.subsystems.exampleSubsystem

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.template.utils.amps
import frc.template.utils.devices.KrakenMotors
import frc.template.utils.devices.OpTalonFX

/**
 * Here you declare your class, it must be named as the subsytem is coded here, for instance,
 * if you're programming an intake, the class has to be named "Intake"
 */
class ExampleSubsystem() : Subsystem {

    // Declare an OpTalonFX, this is the motor controller when using a Kraken Motor
    // It has 2 arguments:
    // First, the motor Id, this has to be an "int"
    // Then the canbus, you can just leave it empty or, if needed, specify a canBus
    private val motorController = OpTalonFX(ExampleConstants.Identification.MOTOR_ID, ExampleConstants.Identification.CAN_BUS)

    /* INITIALIZATION CODE */

    // This code executes when the class is initialized
    init {
        // Calling the motor's configuration
        configureMotorInterface()
    }

    // To set a voltage, you can just call voltageRequest() from your motor controller
    fun setVoltage(voltage: Voltage) {
        motorController.voltageRequest(voltage)
    }

    // In this method you can configure all the necessary aspects of your motor, in this case
    // we're configuring motor outputs and current limits, and then applying the config to the
    // motor controller
    private fun configureMotorInterface() {

        // Creating a talon configuration
        val talonConfig = TalonFXConfiguration()

        // Configuring motor outputs
        talonConfig.MotorOutput = KrakenMotors.configureMotorOutputs(
            ExampleConstants.Configuration.NEUTRAL_MODE_VALUE,
            ExampleConstants.Configuration.INVERTED_VALUE.toInvertedValue()
        )

        // Configuring current limits
        talonConfig.CurrentLimits = KrakenMotors.configureCurrentLimits(
            ExampleConstants.CurrentLimits.MOTOR_CURRENT_LIMIT,
            // DO NOT CHANGE THIS NEXT VALUES
            true,
            120.0.amps
        )

        // Applying the TalonFXConfiguration()
        motorController.applyConfigAndClearFaults(talonConfig)
    }
}