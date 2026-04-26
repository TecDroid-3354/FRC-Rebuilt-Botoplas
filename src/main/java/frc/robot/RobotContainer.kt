// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.events.EventTrigger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.constants.RobotConstants.Autonomous
import frc.robot.constants.RobotConstants
import frc.robot.subsystems.StatesHandler
import frc.robot.subsystems.Superstructure
import frc.template.utils.seconds
import net.tecdroid.util.stateMachine.scheduleCMD
import org.json.simple.parser.ParseException
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.io.IOException

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    private val controller = CommandXboxController(RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_PORT)
    private val superstructure = Superstructure(controller)
    private val statesHandler = StatesHandler(superstructure, controller)

    // Dashboard inputs
    // Set up auto routines
    private val autoChooser: LoggedDashboardChooser<Command?> =
        LoggedDashboardChooser<Command?>("Auto Choices", AutoBuilder.buildAutoChooser())

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        configureAutonomousEventTriggers()
        configureAutonomousRoutines()
        Trigger { isNearShiftEnd() }
            .onTrue(statesHandler.driverControllerRumbleBoth())
            .onFalse(statesHandler.disableControllerRumble())
    }

    fun isNearShiftEnd(): Boolean {
        for (shift in RobotConstants.AllianceShiftsInfo.Shifts) {
            return DriverStation.getMatchTime().seconds in shift
        }
        return false
    }

    fun teleopInitConfig() {
        statesHandler.setDefaultLed()
        superstructure.setDriveDefaultCommand(superstructure.driveFollowingDriverInput())
        superstructure.disableSubsystemsInitCMD().scheduleCMD()
        //if (isFlipped.invoke()) superstructure.resetDrivePoseRed() else superstructure.resetDrivePoseBlue()
    }

    fun autoInitConfig() {
        superstructure.removeDriveDefaultCommand()
        statesHandler.setAutonomousLed()
    }

    fun robotEnabledConfig() {
        superstructure.brakeSubsystems()
    }

    fun robotDisabledConfig() {
        //superstructure.setVisionThrottle(200);
    }

    fun configureAutonomousEventTriggers() {
        EventTrigger(Autonomous.EventTriggerStrings.INTAKE_DEPLOY)
            .onTrue(superstructure.intakeStateCMD().alongWith(statesHandler.setIntakeLed()))
        EventTrigger(Autonomous.EventTriggerStrings.DISABLE_INTAKE_ROLLERS)
            .onTrue(superstructure.disableIntakeRollersCMD().alongWith(statesHandler.setDefaultLed()))
        EventTrigger(Autonomous.EventTriggerStrings.SCORE)
            .onTrue(
                superstructure.scoreStateSequenceAutoRightCMD().alongWith(statesHandler.setShootingLed())
                    .alongWith(WaitCommand(4.5.seconds))
                        .andThen(superstructure.disableSubsystemsAutoCMD().alongWith(statesHandler.setDefaultLed()))
                )
        EventTrigger(Autonomous.EventTriggerStrings.SECOND_PICK_SCORE)
            .onTrue(
                superstructure.scoreStateSequenceAutoRightCMD().alongWith(statesHandler.setShootingLed())
                    .alongWith(WaitCommand(6.0.seconds))
                    .andThen(superstructure.disableSubsystemsAutoCMD().alongWith(statesHandler.setDefaultLed()))
            )

        EventTrigger("Align")
            .onTrue(superstructure.driveTargetingHUBAuto().withTimeout(3.0.seconds))

        EventTrigger(Autonomous.EventTriggerStrings.ENABLE_SHOOTER)
            .onTrue(superstructure.noStateShootOnlyCMD(2000.0))
    }

    /**0
     * Use this method to add all autonomous options.
     * Possible exceptions when PathPlanner is not able to find your Path.
     * Make sure to call every path through its constant in [RobotConstants.AutonomousPathStrings]
     * @throws IOException
     * @throws ParseException
     */
    private fun configureAutonomousRoutines() {
        autoChooser.addDefaultOption("None", Commands.none())

        try {
            autoChooser.addOption("Right Auto", PathPlannerAuto(Autonomous.NameStrings.RIGHT_AUTO))
            autoChooser.addOption("Left Auto", PathPlannerAuto(Autonomous.NameStrings.LEFT_AUTO))
            autoChooser.addOption("2nd Pick Right Auto", PathPlannerAuto(Autonomous.NameStrings.SECOND_PICK_RIGHT_AUT0))
        } catch (e: IOException) {
            throw RuntimeException(e)
        } catch (e: ParseException) {
            throw RuntimeException(e)
        }
    }

    val autonomousCommand: Command?
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()
}
