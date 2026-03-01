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

package frc.robot;

import com.pathplanner.lib.path.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.StatesHandler;
import frc.robot.subsystems.Superstructure;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(RobotConstants.DriverControllerConstants.DRIVER_CONTROLLER_PORT);

    private final Superstructure superstructure = new Superstructure(controller);
    private final StatesHandler statesHandler = new StatesHandler(superstructure, controller);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    //configureAutonomous();
  }

  public void robotEnabledConfig() {
    superstructure.setVisionThrottle(0);
  }

  public void robotDisabledConfig() {
    superstructure.setVisionThrottle(200);
  }

  /**
   * Use this method to add all autonomous options.
   * Possible exceptions when PathPlanner is not able to find your Path.
   * Make sure to call every path through its constant in {@link RobotConstants.AutonomousPathStrings}
   * @throws IOException
   * @throws ParseException
   */
//  private void configureAutonomous() {
//      try {
//          autoChooser.addOption("LT -> One Meter Right", superstructure.followTrajectory(PathPlannerPath.fromPathFile(
//                  RobotConstants.AutonomousPathStrings.LEFT_TRENCH_ONE_METER_RIGHT
//          )));
//
//        autoChooser.addOption("LT -> Five Meter Right While Rotating", superstructure.followTrajectory(PathPlannerPath.fromPathFile(
//                RobotConstants.AutonomousPathStrings.LEFT_TRENCH_FIVE_METERS_RIGHT_WITH_180
//        )));
//
//        autoChooser.addOption("LT -> Through LT to Neutral Zone, Right Trench and Middle Alliance Zone to Right of Alliance Zone",
//                superstructure.followTrajectory(PathPlannerPath.fromPathFile(
//                        RobotConstants.AutonomousPathStrings.LEFT_TRENCH_AROUND_THE_WORLD
//                )));
//
//        autoChooser.addOption("LT -> ZigZag", superstructure.followTrajectory(PathPlannerPath.fromPathFile(
//                RobotConstants.AutonomousPathStrings.ZIG_ZAG
//        )));
//
//        autoChooser.addOption("RT -> Under", superstructure.followTrajectory(PathPlannerPath.fromPathFile(
//                RobotConstants.AutonomousPathStrings.UNDER_RIGHT_TRENCH
//        )));
//      } catch (IOException e) {
//          throw new RuntimeException(e);
//      } catch (ParseException e) {
//          throw new RuntimeException(e);
//      }
//  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
