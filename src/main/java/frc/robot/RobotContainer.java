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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

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
          new ModuleIOTalonFX(SwerveTunerConstants.BackRight));

  private final Intake intake = new Intake();

  // -------------------------------
  // Controllers
  // -------------------------------
  private final CommandXboxController controller =
          new CommandXboxController(Constants.INSTANCE.getDriverControllerId());

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

    // Configure the button bindings
    configureButtonBindings();
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

    controller.a().whileTrue(DriveCommands.wheelRadiusCharacterization(drive));

    controller.b().whileTrue(DriveCommands.feedforwardCharacterization(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
