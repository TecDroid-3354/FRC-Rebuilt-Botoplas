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
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.constants.RobotConstants
import frc.robot.subsystems.StatesHandler
import frc.robot.subsystems.Superstructure
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
    private val autoChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Auto Choices",
        superstructure.getAutoChooser())

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        registerNamedCommands()
        configureAutonomous()
    }

    fun teleopInitConfig() {
        superstructure.setDriveDefaultCommand(superstructure.driveFollowingDriverInput())
        superstructure.resetDrivePose()
    }

    fun robotEnabledConfig() {
        superstructure.brakeSubsystems()
    }

    fun robotDisabledConfig() {
        //superstructure.setVisionThrottle(200);
    }

    /**
     * Use this method to add all autonomous options.
     * Possible exceptions when PathPlanner is not able to find your Path.
     * Make sure to call every path through its constant in [RobotConstants.AutonomousPathStrings]
     * @throws IOException
     * @throws ParseException
     */
    private fun configureAutonomous() {
        autoChooser.addDefaultOption("None", Commands.none())

        try {
            autoChooser.addOption(
                "LT -> One Meter Right", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.LEFT_TRENCH_ONE_METER_RIGHT
                    )
                )
            )

            autoChooser.addOption(
                "LT -> Five Meter Right While Rotating", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.LEFT_TRENCH_FIVE_METERS_RIGHT_WITH_180
                    )
                )
            )

            autoChooser.addOption(
                "LT -> Through LT to Neutral Zone, Right Trench and Middle Alliance Zone to Right of Alliance Zone",
                superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.LEFT_TRENCH_AROUND_THE_WORLD
                    )
                )
            )

            autoChooser.addOption(
                "LT -> ZigZag", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.ZIG_ZAG
                    )
                )
            )

            autoChooser.addOption(
                "RT -> Under", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.UNDER_RIGHT_TRENCH
                    )
                )
            )

            autoChooser.addOption("Right Two Cycles", PathPlannerAuto("RightAutoTwoCycles"))
            autoChooser.addOption("Test", PathPlannerAuto("New Auto"))

//          autoChooser.addOption("Left Two Cycles", );
//            autoChooser.addOption(
//                "RA2", AutoCommand(
//                    "RightAutoTwoCycles",
//                    PathPlannerAuto.getPathGroupFromAutoFile("RightAutoTwoCycles"),
//                    superstructure
//                )
//                    .rightAutoTwoCycles()
//            )

            autoChooser.addOption(
                "RA2 Trench Neutral 1", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.R2C_TRENCH_NEUTRAL_ZONE_1
                    )
                )
            )

            autoChooser.addOption(
                "RA2 Intake 1", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.R2C_INTAKE_1
                    )
                )
            )

            autoChooser.addOption(
                "RA2 Shooter", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.R2C_SHOOT_1
                    )
                )
            )

            autoChooser.addOption(
                "RA2 Trench Neutral 2", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.R2C_TRENCH_NEUTRAL_ZONE_2
                    )
                )
            )

            autoChooser.addOption(
                "RA2 Intake 2", superstructure.followTrajectory(
                    PathPlannerPath.fromPathFile(
                        RobotConstants.AutonomousPathStrings.R2C_INTAKE_2
                    )
                )
            )
        } catch (e: IOException) {
            throw RuntimeException(e)
        } catch (e: ParseException) {
            throw RuntimeException(e)
        }
    }

    private fun registerNamedCommands() {
        NamedCommands.registerCommand("enableIntake", superstructure.intakeStateCMD())
        NamedCommands.registerCommand("disableIntake", superstructure.disableIntake())
        NamedCommands.registerCommand("shoot", superstructure.shootStateSequenceDefaultCMD())
    }

    val autonomousCommand: Command?
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()
}
