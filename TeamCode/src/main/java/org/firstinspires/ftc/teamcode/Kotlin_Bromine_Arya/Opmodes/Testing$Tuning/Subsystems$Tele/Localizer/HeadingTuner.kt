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
    }
    var dif =0.0
    override fun runOpMode() {

        val localizer = TeleLocalizer(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Tune: ", "TIME BETWEEN READ CONFIG")
            telemetry.addData("Rotate Bot 10 times, then set boolean done to true", "")

            localizer.updateHeading()
            val currentHeading = localizer.getRotation()

            telemetry.addData("Heading", currentHeading)

            if (gamepad1.x) {
                localizer.resetHeading()
            }
            if(TeleLocalizer.finished){
               dif = Angle.wrap(Math.PI - currentHeading)
            }
            telemetry.addData("Input this multiplier into TeleLocalizer",dif/3600)

            //TODO(Find a way to Prevent IMU wrap around and have this tuning actually be effective)

            telemetry.update()

        }
    }
}