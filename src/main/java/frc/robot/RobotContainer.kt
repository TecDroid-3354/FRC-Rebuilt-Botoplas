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

import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.events.EventTrigger
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.constants.LedPatterns
import frc.robot.constants.RobotConstants.Autonomous
import frc.robot.constants.RobotConstants
import frc.robot.subsystems.StatesHandler
import frc.robot.subsystems.Superstructure
import frc.template.utils.seconds
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
        LoggedDashboardChooser<Command?>("Auto Choices", superstructure.getAutoChooser())

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        configureAutonomousEventTriggers()
        configureAutonomousRoutines()
    }

    fun teleopInitConfig() {
        superstructure.setLEDPattern(LedPatterns.SolidColors.RED)
        superstructure.setDriveDefaultCommand(superstructure.driveFollowingDriverInput())
        superstructure.disableSubsystemsCMD()
        //if (isFlipped.invoke()) superstructure.resetDrivePoseRed() else superstructure.resetDrivePoseBlue()
    }

    fun autoInitConfig() {
        superstructure.removeDriveDefaultCommand()
    }

    fun robotEnabledConfig() {
        superstructure.brakeSubsystems()
    }

    fun robotDisabledConfig() {
        //superstructure.setVisionThrottle(200);
    }

    fun configureAutonomousEventTriggers() {
        EventTrigger(Autonomous.EventTriggerStrings.INTAKE_DEPLOY)
            .onTrue(superstructure.intakeStateCMD())
        EventTrigger(Autonomous.EventTriggerStrings.DISABLE_INTAKE_ROLLERS)
            .onTrue(superstructure.disableIntakeRollersCMD())
        EventTrigger(Autonomous.EventTriggerStrings.SHOOT_CMD)
            .onTrue(
                superstructure.scoreStateSequenceAutoCMD()
                    .withTimeout(2.0.seconds)
                    .andThen(superstructure.disableSubsystemsCMD())
            )
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
            autoChooser.addOption("Angular offset Test", PathPlannerAuto("Test-Left"))
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
