package frc.robot.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.LedPatterns
import frc.robot.constants.Pattern
import frc.robot.constants.RobotConstants
import frc.robot.constants.RobotConstants.LedControllerConstants.LED_CONTROLLER_PORT
import org.littletonrobotics.junction.AutoLogOutput

class LedController: SubsystemBase() {
    // -------------------------------
    // PRIVATE — Controller Declaration
    // -------------------------------

    // The LED Controller is declared as Spark as its recommended by the REV Blinkin documentation
    // You can find more details at https://docs.revrobotics.com/rev-crossover-products/blinkin/overview
    private val ledController       : Spark = Spark(LED_CONTROLLER_PORT)

    // -------------------------------
    // PRIVATE — Useful variables
    // -------------------------------
    private var currentPWM               : Double = 0.0
    private var targetPattern            : Pattern = LedPatterns.SolidColors.YELLOW

    // -------------------------------
    // PRIVATE — Alerts
    // -------------------------------
    private val ledControllerAlert  : Alert =
        Alert(RobotConstants.LedControllerConstants.LED_CONTROLLER_CONNECTED_ALERTS_FIELD,
            "Rev Blinking in port $LED_CONTROLLER_PORT Disconnected",
            Alert.AlertType.kError
        )

    /**
     * Called upon [frc.robot.subsystems.LedController] creation. Used to initialize a pattern.
     */
    init {
        setPatternCMD(LedPatterns.FixedPalettePatters.RAINBOW_LAVA_PALETTE)
    }

    /**
     * Called every 20ms loop. Used to update alerts.
     */
    override fun periodic() {
        ledControllerAlert.set(ledController.isAlive.not())
        setPattern(targetPattern)
    }

    // ---------------------------------
    // PRIVATE — Runnable LED Control
    // ---------------------------------

    /**
     * A useful method to set a PWM value to the LED controller.
     * The requested value is clamped between -1.0 and 1.0 to avoid malfunctioning
     * [targetPWM] is then updated for telemetry tracking
     * @param value the desired PWM
     */
    private fun setPWM(value: Double) {
        val clampedPWM = MathUtil.clamp(value, -1.0, 1.0)
        ledController.set(clampedPWM)
        currentPWM = clampedPWM
    }

    /**
     * A common method to set an already known pattern.
     * Per documentation, PWM roboRIO values correspond to specific LED Patterns.
     * You can find the whole spreadsheet here: https://docs.revrobotics.com/rev-crossover-products/blinkin/gs/patterns
     * However you the code implementation is here [LedPatterns]
     * If the value was already set, it just returns
     * @param pattern A known pattern from the spreadsheet
     */
    private fun setPattern(pattern: Pattern) {
        if (currentPWM == pattern.pwmValue) return
        setPWM(pattern.pwmValue)
    }

    private fun setTargetPWM(pattern: Pattern) {
        targetPattern = pattern
    }
    // -------------------------------
    // PUBLIC — CMD LED Control
    // -------------------------------

    /**
     * A command version of [setPattern]. [LedController] is added as requirement
     */
    fun setPatternCMD(pattern: Pattern): Command {
        return InstantCommand({ setTargetPWM(pattern) }, this)
    }

    // -------------------------------
    // PUBLIC — Telemetry methods
    // -------------------------------

    /**
     * @return the current PWM value. Useful for debugging
     */
    @AutoLogOutput(key = RobotConstants.LedControllerConstants.TARGET_PWM_FIELD)
    fun getTargetPWM(): Double = currentPWM
}