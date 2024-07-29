package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.Auto.Localizer

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.Localizer

@TeleOp(name = "RRLocalizer", group = "Linear OpMode")
class RRLocalizer : LinearOpMode(){
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = Localizer(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            drive.update()
            telemetry.addData("MajorCommand", Sequencer.MAJORCOMMAND)
            telemetry.addData("x",drive.xPos)
            telemetry.addData("y",drive.yPos)
            telemetry.addData("h",Math.toDegrees(drive.heading.toDouble()))
            telemetry.update()
        }
    }
}