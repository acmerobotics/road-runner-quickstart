package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.PID_Components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer.LOCALIZER
import kotlin.math.cos
import kotlin.math.sin


open class PIDdrive(hwMap: HardwareMap){
    //constants
    private val X: PIDController = PIDController()
    private val Y: PIDController = PIDController()
    private val R: PIDController = PIDController()

    val rightBack: DcMotor = hwMap.get(DcMotorEx::class.java, "rightBack")
    val leftFront: DcMotor = hwMap.get(DcMotorEx::class.java, "leftFront")
    val rightFront: DcMotor = hwMap.get(DcMotorEx::class.java, "rightFront")
    val leftBack: DcMotor = hwMap.get(DcMotorEx::class.java, "leftBack")

    init {
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        X.setPIDF(.05, .0001, .000315, 0.0)
        Y.setPIDF(.05, .0001, .000315, 0.0)
        R.setPIDF(.05, .0001, .000315, 0.0)
    }

    fun driveTo(target: DoubleArray){
        val rx = Math.toRadians(LOCALIZER.headingPos)

        val axial = Y.calculate(LOCALIZER.yPos,target[1])
        val lateral = X.calculate(LOCALIZER.xPos,target[0])
        val turn = 0.0
        // R.calculate(HeadingPosDegrees(), Target[2])

        val rotX = lateral * cos(-rx) - axial * sin(-rx)
        val rotY = lateral * sin(-rx) + axial * cos(-rx)

        leftFront.power = (rotY + rotX + turn) //front left
        leftBack.power = (rotY - rotX + turn) // back left
        rightFront.power = (rotY - rotX - turn) //front right
        rightBack.power = (rotY + rotX - turn) // back right
    }

    fun stopWait(){
        leftFront.power = 0.0
        leftBack.power = 0.0
        rightFront.power = 0.0
        rightBack.power = 0.0
    }
}
