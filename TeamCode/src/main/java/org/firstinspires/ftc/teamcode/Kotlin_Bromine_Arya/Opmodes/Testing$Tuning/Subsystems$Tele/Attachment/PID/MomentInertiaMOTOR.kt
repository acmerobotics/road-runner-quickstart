package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.`Subsystems$Tele`.Attachment.PID

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime

@Config
object Inertia {
    @JvmField
    var angleRotated = 0.0
    @JvmField
    var setEncoderPerGivenDegrees = false
    @JvmField
    var reverseMotor = false
    var momentInertia = .08
    var motorAcceleration = 0.03
}

@TeleOp(name = "Acceleration Test", group = "Linear OpMode")
class MomentInertiaMOTOR : LinearOpMode() {
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        val motor = hardwareMap.get(DcMotorEx::class.java, "shoulder")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        telemetry.addData("Status", "Initializing...")
        telemetry.update()

        val timer = ElapsedTime()
        var initialAngle = 0.0
        var conversion = 0.0
        var started = false
        var lastPower = 0.0
        val motorPowers = ArrayList<Double>()

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {

            if (Inertia.setEncoderPerGivenDegrees) {
                conversion = Inertia.angleRotated / motor.currentPosition
                Inertia.setEncoderPerGivenDegrees = false
            }

            if (Inertia.reverseMotor) {
                motor.direction = DcMotorSimple.Direction.REVERSE
                Inertia.reverseMotor = false
            }

            var currentAngle =0.0
            if (conversion != 0.0) {
                currentAngle = motor.currentPosition * conversion
            }

            if (gamepad1.a && !started) {
                started= true
                initialAngle = currentAngle
                motor.power = 1.0
                timer.reset()
            }
            if(started){

                val powerDelta = motor.power - lastPower
                motorPowers.add(powerDelta)

                // Calculate time elapsed
                val deltaTime = timer.milliseconds() / 1000.0

                // Calculate angular displacement
                val angularDisplacement = Math.toRadians(currentAngle-initialAngle)

                // Calculate angular velocity (radians per second)
                val angularVelocity = angularDisplacement / deltaTime

                // Calculate angular acceleration (radians per second squared)
                val angularAcceleration = angularVelocity / deltaTime

                // Estimate moment of inertia
                val momentInertia = motor.power / angularAcceleration

                if(motor.power >= .98){
                    telemetry.addData("MomentInertia", momentInertia)
                    telemetry.addData("Motor Power Acceleration",  motorPowers.average())
                    motor.power =0.0
                    started = false
                }
                lastPower = motor.power
            }
            telemetry.addData("Encoder:", motor.currentPosition)
            telemetry.addData("Motor Angle (will be 0 until configured):", currentAngle)
            telemetry.update()
        }
    }
}