package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.PIDController


open class Shoulder(hwMap: HardwareMap) {


    enum class ShoulderPos {
        Lowered,
        Raised
    }

    companion object {
        var shoulderPos: ShoulderPos = ShoulderPos.Lowered
        var shoulderLevel: Int = 1
    }

    val shoulder: DcMotorEx = hwMap.get(DcMotorEx::class.java, "shoulder")

    //PID - Constants
    var target: Double = 0.0
    private var ticksPerDegree: Double = 915 / 177.0
    private var powerDrop: Double = -0.004
    private var shoulderPIDF: PIDController = PIDController()

    init {
        shoulderPIDF.setPIDF(.023, .00008, .001175, .123)
        shoulder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shoulder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shoulder.direction = DcMotorSimple.Direction.REVERSE
    }

    fun shoulderAngle(): Double {
        return (shoulder.currentPosition.toDouble() / ticksPerDegree) + 3.0
    }

    //make shoulder target relative to angles
    private fun findTarget() {
        if (shoulderPos == ShoulderPos.Lowered) {
            target = 0.0
        } else if (shoulderPos == ShoulderPos.Raised) {
            when (shoulderLevel) {
                1 -> target = 375.0
                2 -> target = 435.0
                3 -> target = 525.0
                4 -> target = 590.0
                5 -> target = 620.0
            }
        }
    }

    //run consistantly
    fun pidShoulder() {
        findTarget()
        val pidf = shoulderPIDF.calculate(target -shoulder.currentPosition.toDouble())
        shoulder.power =  if (target == 0.0) { powerDrop } else { pidf }
    }


}