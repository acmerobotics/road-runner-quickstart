package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

open class IMU(hwMap: HardwareMap) {

    val imu: IMU = hwMap.get(IMU::class.java, "imu")

    init {
        val orientationOnRobot = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()
    }

    fun resetIMU() {
        imu.resetYaw()
    }

    fun imuOrientation(): Double {
        val orientation: YawPitchRollAngles = imu.robotYawPitchRollAngles
        return orientation.getYaw(AngleUnit.RADIANS)
    }

}