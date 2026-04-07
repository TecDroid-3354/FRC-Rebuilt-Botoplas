@file:Suppress("unused")

package frc.robot.constants

import kotlin.math.abs

data class Pattern(val pwmValue: Double) {
    init {
        require(abs(pwmValue) % 2 != 0.0) { "Rev Blinking PWM value must be odd" }
    }
}

object LedPatterns {

    object FixedPalettePatters {
        val RAINBOW_RAINBOW_PALETTE                     : Pattern = Pattern(-0.99)
        val RAINBOW_PARTY_PALETTE                       : Pattern = Pattern(-0.97)
        val RAINBOW_OCEAN_PALETTE                       : Pattern = Pattern(-0.95)
        val RAINBOW_LAVA_PALETTE                        : Pattern = Pattern(-0.93)
        val RAINBOW_FOREST_PALETTE                      : Pattern = Pattern(-0.91)
        val RAINBOW_WITH_GLITTER                        : Pattern = Pattern(-0.89)
        val CONFETTI                                    : Pattern = Pattern(-0.87)
        val RED_SHOT                                    : Pattern = Pattern(-0.85)
        val BLUE_SHOT                                   : Pattern = Pattern(-0.83)
        val WHITE_SHOT                                  : Pattern = Pattern(-0.81)
        val SINELON_RAINBOW_PALETTE                     : Pattern = Pattern(-0.79)
        val SINELON_PARTY_PALETTE                       : Pattern = Pattern(-0.77)
        val SINELON_OCEAN_PALETTE                       : Pattern = Pattern(-0.75)
        val SINELON_LAVA_PALETTE                        : Pattern = Pattern(-0.73)
        val SINELON_FOREST_PALETTE                      : Pattern = Pattern(-0.71)
        val BEATS_PER_MINUTE_RAINBOW_PALETTE            : Pattern = Pattern(-0.69)
        val BEATS_PER_MINUTE_PARTY_PALETTE              : Pattern = Pattern(-0.67)
        val BEATS_PER_MINUTE_OCEAN_PALETTE              : Pattern = Pattern(-0.65)
        val BEATS_PER_MINUTE_LAVA_PALETTE               : Pattern = Pattern(-0.63)
        val BEATS_PER_MINUTE_FOREST_PALETTE             : Pattern = Pattern(-0.61)
        val FIRE_MEDIUM                                 : Pattern = Pattern(-0.59)
        val FIRE_LARGE                                  : Pattern = Pattern(-0.57)
        val TWINKLES_RAINBOW_PALETTE                    : Pattern = Pattern(-0.55)
        val TWINKLES_PARTY_PALETTE                      : Pattern = Pattern(-0.53)
        val TWINKLES_OCEAN_PALETTE                      : Pattern = Pattern(-0.51)
        val TWINKLES_LAVA_PALETTE                       : Pattern = Pattern(-0.49)
        val TWINKLES_FOREST_PALETTE                     : Pattern = Pattern(-0.47)
        val COLOR_WAVES_RAINBOW_PALETTE                 : Pattern = Pattern(-0.45)
        val COLOR_WAVES_PARTY_PALETTE                   : Pattern = Pattern(-0.43)
        val COLOR_WAVES_OCEAN_PALETTE                   : Pattern = Pattern(-0.41)
        val COLOR_WAVES_LAVA_PALETTE                    : Pattern = Pattern(-0.39)
        val COLOR_WAVES_FOREST_PALETTE                  : Pattern = Pattern(-0.37)
        val LARSON_SCANNER_RED                          : Pattern = Pattern(-0.35)
        val LARSON_SCANNER_GRAY                         : Pattern = Pattern(-0.33)
        val LIGHT_CHASE_RED                             : Pattern = Pattern(-0.31)
        val LIGHT_CHASE_BLUE                            : Pattern = Pattern(-0.29)
        val LIGHT_CHASE_GRAY                            : Pattern = Pattern(-0.27)
        val HEARTBEAT_RED                               : Pattern = Pattern(-0.25)
        val HEARTBEAT_BLUE                              : Pattern = Pattern(-0.23)
        val HEARTBEAT_WHITE                             : Pattern = Pattern(-0.21)
        val HEARTBEAT_GRAY                              : Pattern = Pattern(-0.19)
        val BREATH_RED                                  : Pattern = Pattern(-0.17)
        val BREATH_BLUE                                 : Pattern = Pattern(-0.15)
        val BREATH_GRAY                                 : Pattern = Pattern(-0.13)
        val STROBE_RED                                  : Pattern = Pattern(-0.11)
        val STROBE_BLUE                                 : Pattern = Pattern(-0.09)
        val STROBE_GOLD                                 : Pattern = Pattern(-0.07)
        val STROBE_WHITE                                : Pattern = Pattern(-0.05)
    }

    object ColorOnePatterns {
        val END_TO_END_BLEND_TO_BLACK                   : Pattern = Pattern(-0.03)
        val LARSON_SCANNER                              : Pattern = Pattern(-0.01)
        val LIGHT_CHASE                                 : Pattern = Pattern(0.01)
        val HEARTBEAT_SLOW                              : Pattern = Pattern(0.03)
        val HEARTBEAT_MEDIUM                            : Pattern = Pattern(0.05)
        val HEARTBEAT_FAST                              : Pattern = Pattern(0.07)
        val BREATH_SLOW                                 : Pattern = Pattern(0.09)
        val BREATH_FAST                                 : Pattern = Pattern(0.11)
        val SHOT                                        : Pattern = Pattern(0.13)
        val STROBE                                      : Pattern = Pattern(0.15)
    }

    object ColorTwoPatterns {
        val END_TO_END_BLEND_TO_BLACK                   : Pattern = Pattern(0.17)
        val LARSON_SCANNER                              : Pattern = Pattern(0.19)
        val LIGHT_CHASE                                 : Pattern = Pattern(0.21)
        val HEARTBEAT_SLOW                              : Pattern = Pattern(0.23)
        val HEARTBEAT_MEDIUM                            : Pattern = Pattern(0.25)
        val HEARTBEAT_FAST                              : Pattern = Pattern(0.27)
        val BREATH_SLOW                                 : Pattern = Pattern(0.29)
        val BREATH_FAST                                 : Pattern = Pattern(0.31)
        val SHOT                                        : Pattern = Pattern(0.33)
        val STROBE                                      : Pattern = Pattern(0.35)
    }

    object ColorOneAndTwoPatterns {
        val SPARKLE_COLOR_ONE_TO_COLOR_TWO              : Pattern = Pattern(0.37)
        val SPARKLE_COLOR_TWO_ON_COLOR_ONE              : Pattern = Pattern(0.39)
        val COLOR_GRADIENT_COLOR_ONE_AND_TWO            : Pattern = Pattern(0.41)
        val BEATS_PER_MINUTE_COLOR_ONE_AND_TWO          : Pattern = Pattern(0.43)
        val END_TO_BLEND_COLOR_ONE_TO_TWO               : Pattern = Pattern(0.45)
        val END_TO_BLEND                                : Pattern = Pattern(0.47)
        val COLOR_ONE_AND_TWO_NO_BLENDING               : Pattern = Pattern(0.49)
        val TWINKLES_COLOR_ONE_AND_TWO                  : Pattern = Pattern(0.51)
        val COLOR_WAVES_COLOR_ONE_AND_TWO               : Pattern = Pattern(0.53)
        val SINELON_COLOR_ONE_AND_TWO                   : Pattern = Pattern(0.55)
    }

    object SolidColors {
        val HOT_PINK                                    : Pattern = Pattern(0.57)
        val DARK_RED                                    : Pattern = Pattern(0.59)
        val RED                                         : Pattern = Pattern(0.61)
        val RED_ORANGE                                  : Pattern = Pattern(0.63)
        val ORANGE                                      : Pattern = Pattern(0.65)
        val GOLD                                        : Pattern = Pattern(0.67)
        val YELLOW                                      : Pattern = Pattern(0.69)
        val LAWN_GREEN                                  : Pattern = Pattern(0.71)
        val LIME                                        : Pattern = Pattern(0.73)
        val DARK_GREEN                                  : Pattern = Pattern(0.75)
        val GREEN                                       : Pattern = Pattern(0.77)
        val BLUE_GREEN                                  : Pattern = Pattern(0.79)
        val AQUA                                        : Pattern = Pattern(0.81)
        val SKY_BLUE                                    : Pattern = Pattern(0.83)
        val DARK_BLUE                                   : Pattern = Pattern(0.85)
        val BLUE                                        : Pattern = Pattern(0.87)
        val BLUE_VIOLET                                 : Pattern = Pattern(0.89)
        val VIOLET                                      : Pattern = Pattern(0.91)
        val WHITE                                       : Pattern = Pattern(0.93)
        val GRAY                                        : Pattern = Pattern(0.95)
        val DARK_GRAY                                   : Pattern = Pattern(0.97)
        val BLACK                                       : Pattern = Pattern(0.99)
    }
}