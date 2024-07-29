package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.`Subsystems$Tele`.Attachment

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Shoulder
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Wrist

@Disabled
@TeleOp(name = "WristMath", group = "Linear OpMode")
class Angles: LinearOpMode(){

    @JvmField val resetShoulderEncoder= false

    override fun runOpMode() {
        val shoulder = Shoulder(hardwareMap)
        val wrist = Wrist(hardwareMap)
        telemetry = MultipleTelemetry(telemetry,FtcDashboard.getInstance().telemetry)
        waitForStart()
        while (opModeIsActive()){

            //find 180 degree turn
            telemetry.addData("Shouldder Pos: ",shoulder.shoulder.currentPosition)

            telemetry.addData("Shouldder Angle: ",shoulder.shoulderAngle())
            telemetry.addData("Wrist Target Angle: ",wrist.wristAngle())
            telemetry.addData("Wrist Target Angle: ",wrist.targetpos())
            telemetry.update()
        }

    }
}