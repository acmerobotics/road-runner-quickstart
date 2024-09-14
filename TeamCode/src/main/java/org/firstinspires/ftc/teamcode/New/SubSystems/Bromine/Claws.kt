package org.firstinspires.ftc.teamcode.New.SubSystems.Bromine

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin.SubSystems

class ClawsKt(hardwareMap: HardwareMap) : SubSystems {

    val leftClaw = LeftClaw(hardwareMap)
    val rightClaw = RightClaw(hardwareMap)

    override fun update() {
        leftClaw.update()
        rightClaw.update()
    }

    class LeftClaw(hardwareMap: HardwareMap) : SubSystems, ColorSensorClaw(hardwareMap) {

        enum class States(val servoPose: Double) {
            Open(0.5), Closed(1.0),AutomaticClose(1.0);
        }

        override var state = States.Open
        val servo: Servo = hardwareMap.get(Servo::class.java, "rightClaw")
        override fun update() {
            when (state) {
                States.Open -> servo.position = States.Open.servoPose
                States.Closed -> servo.position = States.Closed.servoPose
                States.AutomaticClose -> {
                    if(checkForRecognition() == Recognition.Left){
                        servo.position = States.Closed.servoPose
                    }
                }
            }
        }

    }

    class RightClaw(hardwareMap: HardwareMap) : SubSystems, ColorSensorClaw(hardwareMap) {

        enum class States(val servoPose: Double) {
            Open(0.5), Closed(1.0),AutomaticClose(1.0);
        }

        override var state = States.Open

        val servo: Servo = hardwareMap.get(Servo::class.java, "leftClaw")
        override fun update() {

            when (state) {
                States.Open -> servo.position = States.Open.servoPose
                States.Closed -> servo.position = States.Closed.servoPose
                States.AutomaticClose -> {
                    if(checkForRecognition() == Recognition.Right){
                        servo.position = States.Closed.servoPose
                    }
                }
            }

        }

    }

}

