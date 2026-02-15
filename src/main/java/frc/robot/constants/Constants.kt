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
package frc.robot.constants

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import frc.template.utils.meters

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
object Constants {
    val simMode: Mode = Mode.SIM
    @JvmField
    val currentMode: Mode = if (RobotBase.isReal()) Mode.REAL else simMode

    val tuningMode = true

    object Field {
        val BLUE_HUB_CENTER_X: Distance = 4.625.meters
        val BLUE_HUB_CENTER_Y: Distance = 4.030.meters
        val RED_HUB_CENTER_X : Distance = 11.920.meters
        val RED_HUB_CENTER_Y : Distance = 4.030.meters
    }

    enum class Mode {
        /** Running on a real robot.  */
        REAL,

        /** Running a physics simulator.  */
        SIM,

        /** Replaying from a log file.  */
        REPLAY
    }

    @JvmField
    val isFlipped: () -> Boolean =
        {
            DriverStation.getAlliance().isPresent
                    && DriverStation.getAlliance().get() == Alliance.Red
        }
}
