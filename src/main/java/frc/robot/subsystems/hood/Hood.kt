package frc.robot.subsystems.hood
import com.ctre.phoenix6.controls.StaticBrake
import com.ctre.phoenix6.signals.InvertedValue //para el clockwise
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.template.utils.devices.NumericId
import frc.template.utils.devices.OpTalonFX
import frc.template.utils.safety.clamp

class Hood() : Subsystem {
    private val motorController: OpTalonFX = OpTalonFX(NumericId(7))


    init {
        motorController.applyConfigAndClearFaults(HoodConstants.Configuration.motorConfig)
        motorController.brake()

    }


    fun setAngle(angle: Angle) {

        val clampedAngle = HoodConstants.PhysicalLimits.limits.coerceIn(angle) as Angle

        // tranforma el angulo multiplicando la reduccion unapply = multiplicacion y apply = divisionn
        val transformedAngle = HoodConstants.PhysicalLimits.REDUCTION.unapply(clampedAngle)

        // le manda el nuevo angulo al motor controller

        //esto ya activa la configuracion hecha para aplicarla
       motorController.getMotorInstance().setControl(HoodConstants.Configuration.motionMagicRequest.withPosition(transformedAngle))

    }
    fun setAngleCmd(angle: Angle) {
        InstantCommand({ setAngle(angle)},this)
    }

    fun breake(){
        motorController.brake()
    }







}