package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Auto.PIDtoPoint

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.BacktrackingKt.Drive
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Actions
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Paths.BlueHelp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer.LOCALIZER
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Claw

@TeleOp(name = "Tuning", group = "Linear OpMode")
class Tuning : LinearOpMode() {

    @Config
    object PIDDRIVETUNE {
        @JvmField var P: DoubleArray = doubleArrayOf(.05, .05, .05)
        @JvmField var I: DoubleArray = doubleArrayOf(0.0001, 0.0001, 0.0001)
        @JvmField var D: DoubleArray = doubleArrayOf(.000315, .000315, .000315)
    }
    override fun runOpMode() {

        val path = BlueHelp(hardwareMap)

        LOCALIZER.mecDrive = Drive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        Claw.clawChoice = Claw.ClawChoice.Both

        Actions.isAuto = true

        Sequencer.MAJORCOMMAND =0

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            LOCALIZER.mecDrive.updatePoseEstimate();
            path.Loop()
            telemetry.addData("MajorCommand", Sequencer.MAJORCOMMAND)
            telemetry.addData("x",LOCALIZER.xPos)
            telemetry.addData("y",LOCALIZER.yPos)
            telemetry.addData("h",LOCALIZER.headingPos)
            telemetry.update()
        }
    }


}
