package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Auto.PIDtoPoint

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.PID_Components.PIDdrive
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Paths.BlueHelp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer.Localizer

@TeleOp(name = "Tuning", group = "Linear OpMode")
class Tuning : LinearOpMode() {

    @Config
    object PIDDRIVETUNE {
        @JvmField var P: DoubleArray = doubleArrayOf(.065, .065, .4)
        @JvmField var I: DoubleArray = doubleArrayOf(0.0001, 0.0001, 0.0001)
        @JvmField var D: DoubleArray = doubleArrayOf(.00035, .00035, .001)
    }
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val startPose = Pose2d(0.0,0.0,0.0)

        val drive = Localizer(hardwareMap, startPose)
        val path = BlueHelp(hardwareMap, drive, startPose)

        path.setPID(PIDDRIVETUNE.P,PIDDRIVETUNE.I,PIDDRIVETUNE.D)

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {

            path.setPID(PIDDRIVETUNE.P,PIDDRIVETUNE.I,PIDDRIVETUNE.D)

            drive.update()
            path.Loop()

            telemetry.addData("MajorCommand", Sequencer.MAJORCOMMAND)
            telemetry.addData("x",drive.xPos)
            telemetry.addData("y",drive.yPos)
            telemetry.addData("h",Math.toDegrees(drive.heading.toDouble()))
            telemetry.addData("boolean",Sequencer.hasReached)
            telemetry.addData("X",PIDdrive.x)
            telemetry.addData("Y",PIDdrive.y)
            telemetry.update()
        }
    }


}
