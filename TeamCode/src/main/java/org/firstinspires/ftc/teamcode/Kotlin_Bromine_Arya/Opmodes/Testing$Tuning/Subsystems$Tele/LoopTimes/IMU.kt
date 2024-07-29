package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.`Subsystems$Tele`.LoopTimes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.JavaUtil
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

@Config
@TeleOp(name = "IMULoopTimeEffeciency", group = "Linear OpMode")
class IMU : LinearOpMode() {

    @JvmField
    var sensorsInUse = 0


    val x = 3.4
    override fun runOpMode() {
        val colorSensor = hardwareMap.get(ColorSensor::class.java, "colorSensor")
        val colorSensor2 = hardwareMap.get(ColorSensor::class.java, "colorSensor2")
        val timer = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val imu: IMU = hardwareMap.get(IMU::class.java, "imu")
        val orientationOnRobot = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()
        waitForStart()

        while (opModeIsActive()) {
            timer.reset()

            fun color1() {
                (colorSensor as NormalizedColorSensor).gain = 2f
                val normalizedColors = (colorSensor as NormalizedColorSensor).normalizedColors
                val color = normalizedColors.toColor()
                val value = JavaUtil.colorToValue(color)
            }

            fun color2() {
                (colorSensor2 as NormalizedColorSensor).gain = 2f
                val normalizedColors2 = (colorSensor2 as NormalizedColorSensor).normalizedColors
                val color2 = normalizedColors2.toColor()
                val value2 = JavaUtil.colorToValue(color2)
            }

            fun imu() {
                val orientation: YawPitchRollAngles = imu.robotYawPitchRollAngles
                orientation.getYaw(AngleUnit.RADIANS)
            }

            when (sensorsInUse) {
                0 -> {}
                1 -> {
                    color1()
                    color2()
                }

                2 -> {
                    color1()
                    color2()
                    imu()
                }
            }

            telemetry.addData("Loop Time", timer.milliseconds())
            telemetry.update()

        }
    }
}