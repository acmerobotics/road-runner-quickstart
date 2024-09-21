package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Opmodes.Camera

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.AprilTagData
import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.TeleLocalizer

@TeleOp(name = "ArduCamAprilTags",group = "Linear OpMode")
class Ardu : LinearOpMode() {

    override fun runOpMode() {
        val localizer = TeleLocalizer(hardwareMap)
        val aprilTagProcessor = AprilTagData(hardwareMap,localizer)

        while (opModeIsActive()){
            localizer.updateHeading()
            val pose = aprilTagProcessor.searchForTag()

            telemetry.addData("Pose x", pose.position.x)
            telemetry.addData("Pose y", pose.position.y)
            telemetry.addData("Pose h", pose.heading)

        }
    }


}