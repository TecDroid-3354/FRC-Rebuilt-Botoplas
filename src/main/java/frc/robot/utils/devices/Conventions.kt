package frc.template.utils.devices

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance

enum class RotationalDirection(private val factor: Int) {
    Counterclockwise(1), Clockwise(-1);

    fun isCounterClockwise(): Boolean = this == Counterclockwise
    fun isClockwise(): Boolean = !isCounterClockwise()

    fun matches(other: RotationalDirection): Boolean = this == other
    fun differs(other: RotationalDirection): Boolean = !matches(other)

    fun rotateClockwise(angle: Angle): Angle = angle * (factor * Clockwise.factor).toDouble()
    fun rotateCounterclockwise(angle: Angle): Angle = angle * (factor * Counterclockwise.factor).toDouble()
    fun rotateWithDirection(angle: Angle, dir: RotationalDirection): Angle = angle * (factor * dir.factor).toDouble()

    fun opposite(): RotationalDirection = if (isCounterClockwise()) Clockwise else Counterclockwise

    fun toSensorDirectionValue(): SensorDirectionValue =
        if (this == Clockwise) SensorDirectionValue.Clockwise_Positive else SensorDirectionValue.CounterClockwise_Positive

    fun toInvertedValue(): InvertedValue =
        if (this == Clockwise) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
}

enum class LongitudinalDirection(private val factor: Int) {
    Front(1), Back(-1);

    fun isFront(): Boolean = this == Front
    fun isBack(): Boolean = this == Back

    fun matches(other: LongitudinalDirection): Boolean = this == other
    fun differs(other: LongitudinalDirection): Boolean = !matches(other)

    fun translateFront(distance: Distance): Distance = distance * (factor * Front.factor).toDouble()
    fun translateBack(distance: Distance): Distance = distance * (factor * Back.factor).toDouble()
    fun translateWithDirection(distance: Distance, dir: LongitudinalDirection): Distance =
        distance * (factor * dir.factor).toDouble()

    fun opposite(): LongitudinalDirection = if (isFront()) Back else Front
}

enum class TransversalDirection(private val factor: Int) {
    Left(1), Right(-1);

    fun isLeft(): Boolean = this == Left
    fun isRight(): Boolean = this == Right

    fun matches(other: TransversalDirection): Boolean = this == other
    fun differs(other: TransversalDirection): Boolean = !matches(other)

    fun translateLeft(distance: Distance): Distance = distance * (factor * Left.factor).toDouble()
    fun translateRight(distance: Distance): Distance = distance * (factor * Right.factor).toDouble()
    fun translateWithDirection(distance: Distance, dir: TransversalDirection): Distance =
        distance * (factor * dir.factor).toDouble()

    fun opposite(): TransversalDirection = if (isLeft()) Left else Right
}

enum class VerticalDirection(private val factor: Int) {
    Up(1), Down(-1);

    fun isUp(): Boolean = this == Up
    fun isDown(): Boolean = this == Down

    fun matches(other: VerticalDirection): Boolean = this == other
    fun differs(other: VerticalDirection): Boolean = !matches(other)

    fun translateUp(distance: Distance): Distance = distance * (factor * Up.factor).toDouble()
    fun translateDown(distance: Distance): Distance = distance * (factor * Down.factor).toDouble()
    fun translateWithDirection(distance: Distance, dir: VerticalDirection): Distance =
        distance * (factor * dir.factor).toDouble()

    fun opposite(): VerticalDirection = if (isUp()) Up else Down
}

open class SpatialConvention(
    val longitudinalDirection: LongitudinalDirection,
    val transversalDirection: TransversalDirection,
    val verticalDirection: VerticalDirection,
    val rotationalDirection: RotationalDirection
) {
    fun rotateClockwise(angle: Angle): Angle = rotationalDirection.rotateClockwise(angle)
    fun rotateCounterclockwise(angle: Angle): Angle = rotationalDirection.rotateCounterclockwise(angle)
    fun translateFront(distance: Distance): Distance = longitudinalDirection.translateFront(distance)
    fun translateBack(distance: Distance): Distance = longitudinalDirection.translateBack(distance)
    fun translateLeft(distance: Distance): Distance = transversalDirection.translateLeft(distance)
    fun translateRight(distance: Distance): Distance = transversalDirection.translateRight(distance)
    fun translateUp(distance: Distance): Distance = verticalDirection.translateUp(distance)
    fun translateDown(distance: Distance): Distance = verticalDirection.translateDown(distance)
}
