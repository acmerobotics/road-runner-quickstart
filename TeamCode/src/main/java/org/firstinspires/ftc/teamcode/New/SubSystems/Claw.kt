package org.firstinspires.ftc.teamcode.New.SubSystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Claws (hardwareMap: HardwareMap):SubSystems {

    val leftClaw = LeftClaw(hardwareMap)
    val rightClaw = RightClaw(hardwareMap)

    override fun update() {
        leftClaw.update()
        rightClaw.update()
    }
}

open class LeftClaw(hardwareMap: HardwareMap): SubSystems {

    enum class States(val servoPose: Double) {
        Open(0.5), Closed(1.0);
    }

    override var state = States.Open

    val servo: Servo = hardwareMap.get(Servo::class.java, "rightClaw")

    override fun update(){

        when(state){
            States.Open -> servo.position = States.Open.servoPose
            States.Closed -> servo.position = States.Open.servoPose
        }

    }

    init {
        servo.direction = Servo.Direction.FORWARD
    }

}

open class RightClaw(hardwareMap: HardwareMap): SubSystems {

    enum class States(val servoPose: Double) {
        Open(0.5), Closed(1.0);
    }
    override var state = States.Open

    val servo: Servo = hardwareMap.get(Servo::class.java, "leftClaw")

    override fun update(){

        when(state){
            States.Open -> servo.position = States.Open.servoPose
            States.Closed -> servo.position = States.Open.servoPose
        }

    }

    init {
        servo.direction = Servo.Direction.FORWARD
    }

}