package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Opmodes.Claw

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.New.SubSystems.Bromine.ClawsKt

@TeleOp(name = "ClawsKT", group = "Linear OpMode")
class ClawOpModeKt : LinearOpMode() {

    override fun runOpMode() {
        val claws = ClawsKt(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.a) {
                claws.rightClaw.state = ClawsKt.RightClaw.States.Closed
                claws.leftClaw.state = ClawsKt.LeftClaw.States.Closed
            }

            if (gamepad1.b) {
                claws.rightClaw.state = ClawsKt.RightClaw.States.Closed
                claws.leftClaw.state = ClawsKt.LeftClaw.States.Open
            }

            if (gamepad1.x) {
                claws.rightClaw.state = ClawsKt.RightClaw.States.Closed
                claws.leftClaw.state = ClawsKt.LeftClaw.States.Closed
            }

            if (gamepad1.y) {
                claws.rightClaw.state = ClawsKt.RightClaw.States.Open
                claws.leftClaw.state = ClawsKt.LeftClaw.States.Closed
            }

            if (gamepad1.right_bumper) {
                claws.rightClaw.state = ClawsKt.RightClaw.States.AutomaticClose
                claws.leftClaw.state = ClawsKt.LeftClaw.States.AutomaticClose
            }

            claws.update()

            telemetry.update()
        }
    }
}
