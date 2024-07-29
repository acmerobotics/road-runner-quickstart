package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.Auto.PIDtoPoint

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Actions
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Pathing.BlueHelp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.Localizer

@TeleOp(name = "Tuning", group = "Linear OpMode")
class Tuning : LinearOpMode() {

    @Config
    object PIDDRIVETUNE {
        @JvmField var P: DoubleArray = doubleArrayOf(.06, .06, .7)
        @JvmField var I: DoubleArray = doubleArrayOf(0.0001, 0.0001, 0.00015)
        @JvmField var D: DoubleArray = doubleArrayOf(.00035, .00035, .0005)
    }
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val startPose = Pose2d(0.0,0.0,0.0)

        val drive = Localizer(hardwareMap, startPose)
        val path = BlueHelp(hardwareMap, drive, startPose)

        path.setPID(PIDDRIVETUNE.P, PIDDRIVETUNE.I, PIDDRIVETUNE.D)

        Sequencer.MAJORCOMMAND =0

        Actions.isAuto = true

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {

            path.setPID(PIDDRIVETUNE.P, PIDDRIVETUNE.I, PIDDRIVETUNE.D)

            drive.update()
            path.Loop()

            telemetry.addData("MajorCommand", Sequencer.MAJORCOMMAND)
            telemetry.addData("x",drive.xPos)
            telemetry.addData("y",drive.yPos)
            telemetry.addData("h",Math.toDegrees(drive.heading.toDouble()))
            telemetry.update()
        }
    }


}
