package frc.robot.constants

import edu.wpi.first.units.measure.Distance
import frc.template.utils.meters

object FieldConstants {
    /**
     * Contains the relevant (x,y) components of the HUB of each Alliance, measured from the center of the HUB.
     */
    object HUB {
        val BLUE_HUB_CENTER_X           : Distance = 4.625  .meters
        val BLUE_HUB_CENTER_Y           : Distance = 4.030  .meters
        val RED_HUB_CENTER_X            : Distance = 11.920 .meters
        val RED_HUB_CENTER_Y            : Distance = 4.030  .meters
    }

    /**
     * Contains the relevant (x,y) components of both Trenches of each Alliance, measured from the middle of the Trench.
     */
    object Trench {
        val BLUE_TRENCH_CENTER_X        : Distance = 4.625.meters
        val RED_TRENCH_CENTER_X         : Distance = 4.625.meters

        val BLUE_ALLIANCE_ZONE_END_X    : Distance = 2.5.meters
        val BLUE_ALLIANCE_ZONE_END_Y    : Distance = 2.5.meters
        val BLUE_NEUTRAL_ZONE_END_X     : Distance = 2.5.meters
        val BLUE_NEUTRAL_ZONE_END_Y     : Distance = 2.5.meters

        val RED_ALLIANCE_ZONE_END_X     : Distance = 2.5.meters
        val RED_ALLIANCE_ZONE_END_Y     : Distance = 2.5.meters
        val RED_NEUTRAL_ZONE_END_X      : Distance = 2.5.meters
        val RED_NEUTRAL_ZONE_END_Y      : Distance = 2.5.meters
    }

}