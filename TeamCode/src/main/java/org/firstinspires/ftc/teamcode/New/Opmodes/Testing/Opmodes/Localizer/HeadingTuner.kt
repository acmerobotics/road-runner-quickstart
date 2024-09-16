package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Opmodes.Localizer

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin.TeleLocalizer

@Disabled
@TeleOp(name = "HeadingTuner", group = "Linear OpMode")
class HeadingTuner : LinearOpMode() {

    //Change these variables once tuned
    @Config
    companion object TeleLocalizer {
        @JvmField
        var timeBetweenRead = 100

        @JvmField
        var Rcoeffecient = 1.0

        @JvmField
        var Lcoeffecient = 1.0

        @JvmField
        var Reset = false
    }

    override fun runOpMode() {
        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        val localizer = TeleLocalizer(hardwareMap)

        var initalHeading = Math.toDegrees(localizer.getRotation())

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Tune: ", "TIME BETWEEN READ CONFIG")
            telemetry.addData("Rotate Bot 10 times right, then reset and repeat for left", "")

            localizer.updateHeading()
            val currentHeading = Math.toDegrees(localizer.getRotation())

            telemetry.addData("Current Heading", currentHeading)

            if (Reset) {
                localizer.resetHeading()
                initalHeading = Math.toDegrees(localizer.getRotation())
            }

            telemetry.addData(
                "If you turned 10 to the right replace Rcoeffecient with ",
                1 - (currentHeading - initalHeading) / 3600
            )
            telemetry.addData(
                "If you turned 10 to the left replace Lcoeffecient with ",
                1 - (currentHeading - initalHeading) / 3600
            )
            telemetry.addData("Reset before turning in the other direction", "")

            telemetry.update()

        }
    }
}