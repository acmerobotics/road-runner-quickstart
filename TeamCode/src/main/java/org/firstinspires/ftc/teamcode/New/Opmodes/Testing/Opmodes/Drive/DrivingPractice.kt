package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Opmodes.Drive//package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.Auto.Localizer

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin.Drive
import org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin.TeleLocalizer

@TeleOp(name = "DrivingPractice", group = "Linear OpMode")
class DrivingPractice : LinearOpMode(){
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val localizer = TeleLocalizer(hardwareMap)
        val drive = Drive(hardwareMap, localizer)

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {

            drive.update()

            telemetry.addData("heading",Math.toDegrees(localizer.heading))
            telemetry.update()
        }
    }
}