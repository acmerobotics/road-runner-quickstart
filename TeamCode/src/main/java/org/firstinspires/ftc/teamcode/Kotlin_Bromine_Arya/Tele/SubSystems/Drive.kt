package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.Localizer
import kotlin.math.cos
import kotlin.math.sin

class Drive(hwMap: HardwareMap, private val localizer: Localizer) {

    //private val autoDrive = autoDriveTo(hwMap)

    private var rx = 0.0

    val rightBack: DcMotor = hwMap.get(DcMotor::class.java, "rightBack")
    val leftFront: DcMotor = hwMap.get(DcMotor::class.java, "leftFront")
    val rightFront: DcMotor = hwMap.get(DcMotor::class.java, "rightFront")
    val leftBack: DcMotor = hwMap.get(DcMotor::class.java, "leftBack")

    companion object {
        var driveType: DriveType = DriveType.Manual
    }

    //loop
    fun drive(gamepadInput: Array<Float>) {
        rx = localizer.heading.toDouble()
        when (driveType) {
            DriveType.Manual -> manualDrive(gamepadInput[0], gamepadInput[1], gamepadInput[2])
            DriveType.Auto -> autoDrive()
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

    private fun manualDrive(axial: Float, lateral: Float, turn: Float) {
        val rotX = axial * cos(-rx) - lateral * sin(-rx)
        val rotY = axial * sin(-rx) + lateral * cos(-rx)

        leftFront.power = (rotY + rotX + turn) //front left
        leftBack.power = (rotY - rotX + turn) // back left
        rightFront.power = (rotY - rotX - turn) //front right
        rightBack.power = (rotY + rotX - turn) // back right
    }

    private fun autoDrive() {
        //autoDrive.AutoDriveTB()
    }

    enum class DriveType {
        Auto, Manual
    }

}


