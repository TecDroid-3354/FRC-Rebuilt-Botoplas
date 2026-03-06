package frc.robot.constants

import edu.wpi.first.units.measure.Distance
import frc.template.utils.meters

enum class FieldZones {
    BLUE_ALLIANCE_ZONE,
    NEUTRAL_ZONE,
    RED_ALLIANCE_ZONE
}

object FieldConstants {
    /**
     * Contains the length and width of the field.
     */
    object Dimensions {
        val FIELD_LENGTH_X              : Distance = 16.540 .meters
        val FIELD_WIDTH_Y               : Distance = 8.070  .meters
    }
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
     * Contains the relevant (x,y) components of both Trenches of each Alliance.
     * Each coordinate is located in the middle of the given trench's width if observed from driver perspective.
     */
    object Trench {
        val BLUE_TRENCH_CENTER_X            : Distance = 4.625.meters
        val RED_TRENCH_CENTER_X             : Distance = 4.625.meters

        val UPPER_TRENCH_Y                  : Distance = 7.030  .meters
        val LOWER_TRENCH_Y                  : Distance = 7.030  .meters

        val NEUTRAL_ZONE_END_DELTA_X        : Distance = 0.5.meters
        val ALLIANCE_ZONE_END_DELTA_X       : Distance = 0.5.meters

        val BLUE_ALLIANCE_ZONE_END_UPPER_X  : Distance = 2.5.meters
        val BLUE_NEUTRAL_ZONE_END_UPPER_X   : Distance = 2.5.meters

        val BLUE_ALLIANCE_ZONE_END_LOWER_X  : Distance = 2.5.meters
        val BLUE_NEUTRAL_ZONE_END_LOWER_X   : Distance = 2.5.meters

        val RED_ALLIANCE_ZONE_END_UPPER_X   : Distance = 2.5.meters
        val RED_NEUTRAL_ZONE_END_UPPER_X    : Distance = 2.5.meters

        val RED_ALLIANCE_ZONE_END_LOWER_X   : Distance = 2.5.meters
        val RED_NEUTRAL_ZONE_END_LOWER_X    : Distance = 2.5.meters
    }

}