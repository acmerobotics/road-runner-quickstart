package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Auto.Localizer

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.BacktrackingKt.Drive
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer.LOCALIZER

@TeleOp
class RRLocalizer : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        LOCALIZER.mecDrive = Drive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            LOCALIZER.mecDrive.updatePoseEstimate();
            telemetry.addData("MajorCommand", Sequencer.MAJORCOMMAND)
            telemetry.addData("x",LOCALIZER.xPos)
            telemetry.addData("y",LOCALIZER.yPos)
            telemetry.addData("h",LOCALIZER.headingPos)
            telemetry.update()
        }
    }
}