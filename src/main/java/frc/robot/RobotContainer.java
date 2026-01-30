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
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveTunerConstants;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Vision vision;

  // -------------------------------
  // Subsystems
  // -------------------------------
  private final Drive drive = new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(SwerveTunerConstants.FrontLeft),
                  new ModuleIOTalonFX(SwerveTunerConstants.FrontRight),
                  new ModuleIOTalonFX(SwerveTunerConstants.BackLeft),
                  new ModuleIOTalonFX(SwerveTunerConstants.BackRight)
          );

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

    private final Intake intake = new Intake();

    // -------------------------------
    // Controllers
    // -------------------------------
    private final CommandXboxController controller =
            new CommandXboxController(Constants.INSTANCE.getDriverControllerId());

  private List<Waypoint> waypoints;

  PathConstraints constraints = new PathConstraints(3.5, 3.5,
          3 * Math.PI, 4 * Math.PI); // The constraints for this path.

  private PathPlannerPath underTheTrenchTestPathOnTheFly;

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
        vision =
                new Vision(
                        drive::addVisionMeasurement,
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {});
        break;
    }
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    configureAutonomous();

    configureOnTheFlyWaypoints();

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

        autoChooser.addOption("RT -> Under", drive.followTrajectory(PathPlannerPath.fromPathFile(
                Constants.AutonomousPaths.UNDER_RIGHT_TRENCH
        )));
      } catch (IOException e) {
          throw new RuntimeException(e);
      } catch (ParseException e) {
          throw new RuntimeException(e);
      }
  }

  private void configureOnTheFlyWaypoints() {
      waypoints = PathPlannerPath.waypointsFromPoses(
              new Pose2d(3.172, 0.677, Rotation2d.fromDegrees(0)),
              new Pose2d(7.019, 0.677, Rotation2d.fromDegrees(0))
      );

    // Create the path using the waypoints created above
      underTheTrenchTestPathOnTheFly = new PathPlannerPath(
              waypoints,
              constraints,
              null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
              new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );

    // Prevent the path from being flipped if the coordinates are already correct
      underTheTrenchTestPathOnTheFly.preventFlipping = !Constants.isFlipped.invoke();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // -------------------------------
    // Drive (default command)
    // -------------------------------
    drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                    drive,
                    () -> -controller.getLeftY() * 0.8 ,
                    () -> -controller.getLeftX() * 0.8 ,
                    () -> controller.getRightX() * 0.6 * -1.0
            )
    );
                    () -> -controller.getLeftY() * 0.8,
                    () -> -controller.getLeftX() * 0.8,
                    () -> controller.getRightX() * 0.6));

    // -------------------------------
    // Intake bindings
    // -------------------------------

    // Right bumper → deploy intake + run rollers while held
    controller
            .rightBumper()
            .whileTrue(Commands.run(intake::enableIntake, intake))
            .onFalse(Commands.runOnce(intake::stopIntake, intake));

    // Left bumper → force retract + stop
    controller
            .leftBumper()
            .onTrue(Commands.runOnce(intake::stopIntake, intake));

    // Reset gyro to 0° when Start button is pressed
    controller.start().onTrue(
            Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                    drive
                )
            );
    // -------------------------------
    // Other bindings
    // -------------------------------

    // Reset gyro to 0° when Start button is pressed
    controller
            .start()
            .onTrue(
                    Commands.runOnce(
                            () ->
                                    drive.setPose(
                                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                            drive));

      try {
          controller.a().onTrue
                  (drive.followTrajectory(PathPlannerPath.fromPathFile(Constants.AutonomousPaths.LEFT_TRENCH_FIVE_METERS_RIGHT_WITH_180)))
                  .onFalse(new InstantCommand(() -> {if (drive.getCurrentCommand() != null) drive.getCurrentCommand().cancel();}));

          controller.b().onTrue
                          (drive.followTrajectory(underTheTrenchTestPathOnTheFly))
                  .onFalse(new InstantCommand(() -> {if (drive.getCurrentCommand() != null) drive.getCurrentCommand().cancel();}));

          controller.y().onTrue
                          (AutoBuilder.pathfindToPose(
                                  new Pose2d(10.7, 7.45, Rotation2d.fromDegrees(180)),
                                  constraints))
                  .onFalse(new InstantCommand(() -> {if (drive.getCurrentCommand() != null) drive.getCurrentCommand().cancel();}));

          controller.x().onTrue
                          (AutoBuilder.pathfindThenFollowPath(
                                  PathPlannerPath.fromPathFile(Constants.AutonomousPaths.LEFT_TRENCH_FIVE_METERS_RIGHT_WITH_180),
                                  constraints))
                  .onFalse(new InstantCommand(() -> {if (drive.getCurrentCommand() != null) drive.getCurrentCommand().cancel();}));
      } catch (IOException e) {
          throw new RuntimeException(e);
      } catch (ParseException e) {
          throw new RuntimeException(e);
      }

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
