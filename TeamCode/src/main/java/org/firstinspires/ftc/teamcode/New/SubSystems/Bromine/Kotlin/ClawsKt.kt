package org.firstinspires.ftc.teamcode.New.SubSystems.Bromine.Kotlin

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.New.SubSystems.SubSystems

class ClawsKt(hardwareMap: HardwareMap) : SubSystems {

    val leftClaw = LeftClawKt(hardwareMap)
    val rightClaw = RightClawKt(hardwareMap)

    override fun update() {
        leftClaw.update()
        rightClaw.update()
    }

    class LeftClawKt(hardwareMap: HardwareMap) : SubSystems, ColorSensorClaw(hardwareMap) {

        enum class LeftClawStates(val servoPose: Double) {
            Open(0.5), Closed(1.0),AutomaticClose(1.0);
        }

        override var state = LeftClawStates.Open
        val servo: Servo = hardwareMap.get(Servo::class.java, "rightClaw")
        override fun update() {
            when (state) {
                LeftClawStates.Open -> servo.position = LeftClawStates.Open.servoPose
                LeftClawStates.Closed -> servo.position = LeftClawStates.Closed.servoPose
                LeftClawStates.AutomaticClose -> {
                    if(checkForRecognition() == Recognition.Left){
                        servo.position = LeftClawStates.Closed.servoPose
                    }
                }
            }
        }

    }

    class RightClawKt(hardwareMap: HardwareMap) : SubSystems, ColorSensorClaw(hardwareMap) {

        enum class RightClawStates(val servoPose: Double) {
            Open(0.5), Closed(1.0),AutomaticClose(1.0);
        }

        override var state = RightClawStates.Open

        val servo: Servo = hardwareMap.get(Servo::class.java, "leftClaw")
        override fun update() {

            when (state) {
                RightClawStates.Open -> servo.position = RightClawStates.Open.servoPose
                RightClawStates.Closed -> servo.position = RightClawStates.Closed.servoPose
                RightClawStates.AutomaticClose -> {
                    if(checkForRecognition() == Recognition.Right){
                        servo.position = RightClawStates.Closed.servoPose
                    }
                }
            }

        }

    }

}

