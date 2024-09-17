package org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class VerticalSlides(hardwareMap: HardwareMap) : SubSystems {
    enum class States {
        Lowered, Basket, Hang, Clips
    }

    override val state = States.Lowered
    //in inches
    var targetHeight = 0
    val leftSlide = LeftSlide(hardwareMap)
    val rightSlide = RightSlide(hardwareMap)

    override fun update() {

        targetHeight = when (state) {
            States.Lowered -> 0
            States.Basket -> 40
            States.Hang -> 60
            States.Clips -> 50
        }

        rightSlide.update(targetHeight)
        leftSlide.update(targetHeight)
    }

    inner class LeftSlide(hardwareMap: HardwareMap) {

        private val pidController=  PIDFcontroller(PIDParams(0.0,0.0,0.0,0.0))
        private val leftSlide : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftSlide")


        init {
            leftSlide.direction = DcMotorSimple.Direction.FORWARD
            leftSlide.mode=DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        fun update(target : Int) {
            pidController.calculate((target - leftSlide.currentPosition).toDouble())
            //have different pid params here
        }

    }
    inner class RightSlide(hardwareMap: HardwareMap) {

        private val pidController=  PIDFcontroller(PIDParams(0.0,0.0,0.0,0.0))
        private val rightSlide : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftSlide")

        init {
            rightSlide.direction = DcMotorSimple.Direction.FORWARD
            rightSlide.mode=DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        fun update(target : Int) {
            pidController.calculate((target - rightSlide.currentPosition).toDouble())

            //have different pid params here
        }

    }

}
