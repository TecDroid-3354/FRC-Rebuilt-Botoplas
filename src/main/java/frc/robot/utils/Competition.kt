package frc.template.utils

import edu.wpi.first.wpilibj.DriverStation

object MatchStatus {
    val alliance: DriverStation.Alliance
        get() = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)

    val isBlueAlliance: Boolean
        get() = alliance == DriverStation.Alliance.Blue

    val isRedAlliance: Boolean
        get() = alliance == DriverStation.Alliance.Red
}