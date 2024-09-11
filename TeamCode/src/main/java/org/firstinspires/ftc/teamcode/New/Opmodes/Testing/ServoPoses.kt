package org.firstinspires.ftc.teamcode.New.Opmodes.Testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo

//TODO add @Config when using
class ServoPoses: LinearOpMode() {

    companion object{
        @JvmField var position = 0.0
        @JvmField var direction: Servo.Direction = Servo.Direction.FORWARD
    }

    override fun runOpMode() {

        //TODO add name
        val servo :Servo = hardwareMap.get(Servo::class.java, "")

        while (opModeIsActive()){

            servo.position = position
            servo.direction = direction

        }

    }


}