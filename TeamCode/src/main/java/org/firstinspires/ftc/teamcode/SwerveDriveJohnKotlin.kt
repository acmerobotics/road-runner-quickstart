package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.IMU
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.atan
import kotlin.math.PI
import kotlin.time.times

class SwerveDrive (hwMap: HardwareMap) {

    val ticksperrevo = 0.0
    var power = 0.0
    val imu: IMU = IMU(hwMap)
    var Rot: Double = 0.0

    enum class Direction{
        Forward, Backwards, Stopped
    }

    companion object{
        var direction: Direction = Direction.Forward
    }

    val rightBackDrive: DcMotor = hwMap.get(DcMotor::class.java, "rightBack")
    val leftBackDrive: DcMotor = hwMap.get(DcMotor::class.java, "leftBack")
    val rightFrontDrive: DcMotor = hwMap.get(DcMotor::class.java, "rightFront")
    val leftFrontDrive: DcMotor = hwMap.get(DcMotor::class.java, "leftFront")

    val rightBackRot: DcMotor = hwMap.get(DcMotor::class.java, "rightBackRot")
    val leftBackRot: DcMotor = hwMap.get(DcMotor::class.java, "leftBackRot")
    val rightFrontRot: DcMotor = hwMap.get(DcMotor::class.java, "rightFrontRot")
    val leftFrontRot: DcMotor = hwMap.get(DcMotor::class.java, "leftFrontRot")


    fun drive (gamepadInput: Array<Float>){
        Rot = imu.imuOrientation()
        val Axial = gamepadInput[0] * cos(-Rot) - gamepadInput[1] * sin(-Rot)
        val Lateral = gamepadInput[0] * sin(-Rot) + gamepadInput[1] * cos(-Rot)

        val angle: Double = atan(Lateral/Axial)

        //Drive Motors

        if (Axial == 0.0 && Lateral == 0.0){
            direction = Direction.Stopped
        }
        else if (Axial >= 0) {
            direction = Direction.Forward
        }
        else {
            direction = Direction.Backwards
        }

        power = when (direction) {
            Direction.Forward -> 1.0
            Direction.Backwards -> -1.0
            Direction.Stopped -> 0.0
        }
        leftFrontDrive.power = (power)
        rightFrontDrive.power = (power)
        rightBackDrive.power = (power)
        leftBackDrive.power = (power)

        val ticks = (angle/(2 * PI) * ticksperrevo)

        rightBackRot.targetPosition = ticks.toInt()
        leftBackRot.targetPosition = ticks.toInt()
        leftFrontRot.targetPosition = ticks.toInt()
        rightFrontRot.targetPosition = ticks.toInt()

    }
    init {
        leftBackDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftFrontDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBackDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFrontDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBackDrive.direction = DcMotorSimple.Direction.REVERSE
        leftFrontDrive.direction = DcMotorSimple.Direction.REVERSE

        leftBackRot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftFrontRot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBackRot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFrontRot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBackRot.direction = DcMotorSimple.Direction.FORWARD
        leftFrontRot.direction = DcMotorSimple.Direction.FORWARD
        leftBackRot.direction = DcMotorSimple.Direction.FORWARD
        leftFrontRot.direction = DcMotorSimple.Direction.FORWARD
    }
}