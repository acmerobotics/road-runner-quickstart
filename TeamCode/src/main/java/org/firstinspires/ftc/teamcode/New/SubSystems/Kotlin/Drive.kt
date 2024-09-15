package org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos
import kotlin.math.sin

//todo Make Java version
class Drive(hwMap: HardwareMap, private val localizer: TeleLocalizer) : SubSystems {
    enum class States {
        Manual, Auto
    }

    override val state = States.Manual

    val rightBack: DcMotor = hwMap.get(DcMotor::class.java, "rightBack")
    val leftFront: DcMotor = hwMap.get(DcMotor::class.java, "leftFront")
    val rightFront: DcMotor = hwMap.get(DcMotor::class.java, "rightFront")
    val leftBack: DcMotor = hwMap.get(DcMotor::class.java, "leftBack")

    override fun update(gamepadInput: ArrayList<Float>) {
        localizer.updateHeading()
        val rx = localizer.heading

        when (state) {
            States.Auto -> {

            }

            States.Manual -> {
                val (lateral, axial, turn) = gamepadInput
                val h = -rx
                val rotX = axial * cos(h) - lateral * sin(h)
                val rotY = axial * sin(h) + lateral * cos(h)

                leftFront.power = (rotY + rotX + turn) //front left
                leftBack.power = (rotY - rotX + turn) // back left
                rightFront.power = (rotY - rotX - turn) //front right
                rightBack.power = (rotY + rotX - turn) // back right}
            }
        }
    }

    init {
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE
    }

}