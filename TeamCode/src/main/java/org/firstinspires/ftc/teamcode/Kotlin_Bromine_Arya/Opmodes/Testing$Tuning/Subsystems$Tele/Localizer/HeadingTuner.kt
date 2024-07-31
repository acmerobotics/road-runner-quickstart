package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.`Subsystems$Tele`.LoopTimes

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Angle
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.TeleLocalizer

@TeleOp(name = "HeadingTuner", group = "Linear OpMode")
class HeadingTuner : LinearOpMode() {

    @Config
    object TeleLocalizer {
        @JvmField
        val timeBetweenRead = 100

        @JvmField
        val finished = false

        @JvmField
        val Rcoeffecient = 1.0

        @JvmField
        val Lcoeffecient = 1.0


        @JvmField
        val Reset = false
    }
    var dif =0.0
    override fun runOpMode() {

        val localizer = TeleLocalizer(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Tune: ", "TIME BETWEEN READ CONFIG")
            telemetry.addData("Rotate Bot 10 times either direction, then set boolean done to true", "")

            localizer.updateHeading()
            val currentHeading = localizer.getRotation()

            telemetry.addData("Heading", currentHeading)

            if (gamepad1.x) {
                localizer.resetHeading()
            }

            //TODO(Set start offset, have difference be subtracting by end offset, have degrees reflect that too, implement reset variable)

            telemetry.addData("If you turned 10 to the right replace Rcoeffecient with ",dif/3600)
            telemetry.addData("If you turned 10 to the left replace Lcoeffecient with ",dif/3600)
            telemetry.addData("Set reset to true before measuring the other side","",)

            telemetry.update()

        }
    }
}