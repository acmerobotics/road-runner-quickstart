package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Opmodes.Claw

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.New.SubSystems.Bromine.Kotlin.ClawsKt

@TeleOp(name = "ClawsKT", group = "Linear OpMode")
class ClawOpModeKt : LinearOpMode() {

    override fun runOpMode() {
        val claws = ClawsKt(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.a) {
                claws.rightClaw.state = ClawsKt.RightClawKt.RightClawStates.Closed
                claws.leftClaw.state = ClawsKt.LeftClawKt.LeftClawStates.Closed
            }

            if (gamepad1.b) {
                claws.rightClaw.state = ClawsKt.RightClawKt.RightClawStates.Closed
                claws.leftClaw.state = ClawsKt.LeftClawKt.LeftClawStates.Open
            }

            if (gamepad1.x) {
                claws.rightClaw.state = ClawsKt.RightClawKt.RightClawStates.Closed
                claws.leftClaw.state = ClawsKt.LeftClawKt.LeftClawStates.Closed
            }

            if (gamepad1.y) {
                claws.rightClaw.state = ClawsKt.RightClawKt.RightClawStates.Open
                claws.leftClaw.state = ClawsKt.LeftClawKt.LeftClawStates.Closed
            }

            if (gamepad1.right_bumper) {
                claws.rightClaw.state = ClawsKt.RightClawKt.RightClawStates.AutomaticClose
                claws.leftClaw.state = ClawsKt.LeftClawKt.LeftClawStates.AutomaticClose
            }

            claws.update()

            telemetry.update()
        }
    }
}
