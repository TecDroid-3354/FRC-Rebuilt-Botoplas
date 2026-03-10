package frc.robot.commands.AutoCommands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.constants.RobotConstants
import frc.robot.subsystems.Superstructure
import frc.template.utils.seconds
import org.littletonrobotics.junction.AutoLogOutput
import java.util.function.BooleanSupplier

class AutoCommand(val autoName: String, val paths: List<PathPlannerPath>, val superstructure: Superstructure): SequentialCommandGroup() {

    fun resetAutoPoseCMD(): Command = superstructure.resetDrivePose(paths[0].pathPoses.first())
    fun enableIntakeCMD(): Command = superstructure.intakeStateCMD()
    fun disableIntakeCMD(): Command = superstructure.disableIntake()
    fun disableSubsystems(): Command = superstructure.disableSubsystems()
    fun shootCMD(): Command = superstructure.shootStateSequenceDefaultCMD()
    fun isShooterActive(): BooleanSupplier = superstructure.isShooterActive()
    fun PathPlannerPath.followPath(): Command = superstructure.followTrajectory(this)
    fun PathPlannerPath.followPathAndExecute(command: Command): Command = this.followPath().alongWith(command)

    fun rightAutoTwoCycles(): Command {
        return SequentialCommandGroup(
            AutoBuilder.followPath(getPathFromFile(RobotConstants.AutonomousPathStrings.R2C_TRENCH_NEUTRAL_ZONE_1)),
            AutoBuilder.followPath(getPathFromFile(RobotConstants.AutonomousPathStrings.R2C_INTAKE_1)),
            AutoBuilder.followPath(getPathFromFile(RobotConstants.AutonomousPathStrings.R2C_SHOOT_1)),
            AutoBuilder.followPath(getPathFromFile(RobotConstants.AutonomousPathStrings.R2C_TRENCH_NEUTRAL_ZONE_2)),
            AutoBuilder.followPath(getPathFromFile(RobotConstants.AutonomousPathStrings.R2C_INTAKE_2)),
            AutoBuilder.followPath(getPathFromFile(RobotConstants.AutonomousPathStrings.R2C_SHOOT_2))
        )
    }

    @AutoLogOutput(key = "Auto/Running Command")
    fun getAutoLog(): String {
        return rightAutoTwoCycles().name
    }

    private fun getPathFromFile(fileName: String): PathPlannerPath? {
        try {
            return PathPlannerPath.fromPathFile(fileName)
        } catch (e: Exception) {

        }
        Alert("Papi que putas", Alert.AlertType.kError).set(true)
        return null
    }
}