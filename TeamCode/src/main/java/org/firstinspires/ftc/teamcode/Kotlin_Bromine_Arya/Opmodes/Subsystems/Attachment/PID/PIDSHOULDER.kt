package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Subsystems.Attachment.PID

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Util.PIDController

@Disabled
@TeleOp(name = "PIDSHOULDER", group = "Linear OpMode")
class PIDSHOULDER : LinearOpMode() {
    @Config
    object bruh{
        @JvmField var target= 0.0
    }
    val shoulder: PIDController = PIDController()
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        shoulder.setPIDF(.0235, .00008, .00115, .12)

        val Shoulder = hardwareMap.get(DcMotorEx::class.java, "shoulder")

        Shoulder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Shoulder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        Shoulder.direction = DcMotorSimple.Direction.REVERSE

        val powerDrop: Double = -0.004

        waitForStart()

        while (opModeIsActive()) {
            val PidfPower =
                shoulder.calculate(Shoulder.currentPosition.toDouble(), bruh.target.toDouble())

            Shoulder.power = if (bruh.target.toInt() == 0) {
                powerDrop
            } else {
                PidfPower
            }

            telemetry.addData("powerD", Shoulder.power)
            telemetry.update()
        }
    }

}
