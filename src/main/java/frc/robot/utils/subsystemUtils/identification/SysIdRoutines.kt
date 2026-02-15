package frc.robot.utils.subsystemUtils.identification

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.utils.subsystemUtils.generic.SysIdSubsystem
import java.util.function.Consumer

class SysIdRoutines(
    val quasistaticForward: Command,
    val quasistaticBackward: Command,
    val dynamicForward: Command,
    val dynamicBackward: Command
) {}

/**
 * Motor variables and setVoltage methods are called separetaly to enable the use of
 * protected variables within [SysIdSubsystem]. This is to keep only usable methods visible
 * when accessing a [SysIdSubsystem] object.
 */
class GenericSysIdRoutine(val name: String,
                          val subsystem: SysIdSubsystem,
                          var forwardsRunningCondition : () -> Boolean = { true },
                          var backwardsRunningCondition : () -> Boolean = { true },
                          var motorPower: () -> Double,
                          var motorPosition: () -> Angle,
                          var motorVelocity: () -> AngularVelocity,
                          subsystemSetVoltage : Consumer<Voltage>,
    ) {
    private val position: MutAngle = Radians.mutable(0.0)
    private val velocity: MutAngularVelocity = RadiansPerSecond.mutable(0.0)
    private val voltage: MutVoltage = Volts.mutable(0.0)

    private val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            subsystemSetVoltage,
            { log: SysIdRoutineLog ->
                log.motor(name)
                    .voltage(voltage.mut_replace(RobotController.getBatteryVoltage() * motorPower.invoke(), Volts))
                    .angularPosition(position.mut_replace(motorPosition.invoke()))
                    .angularVelocity(velocity.mut_replace(motorVelocity.invoke()))
            },
            subsystem
        )
    )

    private fun createQuasistaticTest(direction: SysIdRoutine.Direction) = routine.quasistatic(direction)
    private fun createDynamicTest(direction: SysIdRoutine.Direction) = routine.dynamic(direction)

    fun createTests() = SysIdRoutines(
        quasistaticForward = createQuasistaticTest(SysIdRoutine.Direction.kForward).onlyWhile(forwardsRunningCondition),
        quasistaticBackward = createQuasistaticTest(SysIdRoutine.Direction.kReverse).onlyWhile(backwardsRunningCondition),
        dynamicForward = createDynamicTest(SysIdRoutine.Direction.kForward).onlyWhile(forwardsRunningCondition),
        dynamicBackward = createDynamicTest(SysIdRoutine.Direction.kReverse).onlyWhile(backwardsRunningCondition),
    )
}
