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

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveTunerConstants;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
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
  private final Vision vision;

  private final Drive drive = new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(SwerveTunerConstants.FrontLeft),
                  new ModuleIOTalonFX(SwerveTunerConstants.FrontRight),
                  new ModuleIOTalonFX(SwerveTunerConstants.BackLeft),
                  new ModuleIOTalonFX(SwerveTunerConstants.BackRight)
          );

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController controller = new CommandXboxController(Constants.INSTANCE.getDriverControllerId());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation),
                new VisionIOLimelight(camera3Name, drive::getRotation));
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement,
                new VisionIO() {}, new VisionIO() {},
                new VisionIO() {}, new VisionIO() {});
        break;
    }
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutonomous();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to add all autonomous options.
   * Possible exceptions when PathPlanner is not able to find your Path.
   * Make sure to call every path through its constant in {@link Constants.AutonomousPaths}
   * @throws IOException
   * @throws ParseException
   */
  private void configureAutonomous() {
      try {
          autoChooser.addOption("LT -> One Meter Right", drive.followTrajectory(PathPlannerPath.fromPathFile(
                  Constants.AutonomousPaths.LEFT_TRENCH_ONE_METER_RIGHT
          )));

        autoChooser.addOption("LT -> Five Meter Right While Rotating", drive.followTrajectory(PathPlannerPath.fromPathFile(
                Constants.AutonomousPaths.LEFT_TRENCH_FIVE_METERS_RIGHT_WITH_180
        )));

        autoChooser.addOption("LT -> Through LT to Neutral Zone, Right Trench and Middle Alliance Zone to Right of Alliance Zone",
                drive.followTrajectory(PathPlannerPath.fromPathFile(
                        Constants.AutonomousPaths.LEFT_TRENCH_AROUND_THE_WORLD
                )));

        autoChooser.addOption("LT -> ZigZag", drive.followTrajectory(PathPlannerPath.fromPathFile(
                Constants.AutonomousPaths.ZIG_ZAG
        )));
      } catch (IOException e) {
          throw new RuntimeException(e);
      } catch (ParseException e) {
          throw new RuntimeException(e);
      }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Joystick drive command
    drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                    drive,
                    () -> -controller.getLeftY() * 0.8 ,
                    () -> -controller.getLeftX() * 0.8 ,
                    () -> controller.getRightX() * 0.6 * -1.0
            )
    );

    // Reset gyro to 0° when Start button is pressed
    controller.start().onTrue(
            Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                    drive
                )
            );

    controller.a().whileTrue(DriveCommands.wheelRadiusCharacterization(drive));

    controller.b().whileTrue(DriveCommands.feedforwardCharacterization(drive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
