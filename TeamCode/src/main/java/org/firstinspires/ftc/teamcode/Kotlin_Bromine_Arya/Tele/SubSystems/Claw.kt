package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Actions
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer

class Claw(hwMap: HardwareMap) : Shoulder(hwMap) {


    enum class ClawContainer {
        Full,
        HalfFull,
        Empty
    }

    enum class ClawChoice {
        Left,
        Right,
        Both
    }

    enum class ClawState {
        CLOSE,
        OPEN
    }

    var claw: Servo = hwMap.get(Servo::class.java, "claw")
    var claw2: Servo = hwMap.get(Servo::class.java, "claw2")

    init {
        claw2.direction = Servo.Direction.REVERSE
        claw2.scaleRange(.4, .83)
        claw.scaleRange(.67, 1.0)
    }

    companion object {
        var clawChoice: ClawChoice = ClawChoice.Both
        var clawState: ClawState = ClawState.OPEN
        var clawContainer: ClawContainer = ClawContainer.Empty
        var notBothClaws = false
    }

    fun manual() {
        when (clawState) {
            ClawState.CLOSE ->
                when (clawChoice) {
                    ClawChoice.Both -> {
                        claw.position = 1.0
                        claw2.position = 1.0
                    }

                    ClawChoice.Right -> claw.position = 1.0
                    ClawChoice.Left -> claw2.position = 1.0
                }

            ClawState.OPEN ->
                when (clawChoice) {
                    ClawChoice.Both -> {
                        claw.position = 0.0
                        claw2.position = 0.0
                    }

                    ClawChoice.Right -> claw.position = 0.0
                    ClawChoice.Left -> claw2.position = 0.0
                }
        }
    }

    private fun fullSequence() {
        clawContainer = ClawContainer.Full
        Wrist.wristPos = Wrist.WristPos.Raised
        Lights.lightScenario = Lights.LightScenario.ClawFull
        Rumble.RumbleController = Rumble.Grabbed
    }

    private fun semiSequence() {
        clawContainer = ClawContainer.HalfFull
        if (shoulderAngle() >= 90) {
            Lights.lightScenario = Lights.LightScenario.ClawSemiFull
        }
    }


    //weird logic to determine whether the claw has reached it's designated status

    fun clawPos() {
        val rightClaw = claw.position > .3
        val leftClaw = claw2.position > .3
        when (notBothClaws) {
            true -> if (rightClaw || leftClaw) {
                if (Actions.isAuto) {
                    fullSequence()
                } else {
                    semiSequence()
                }
            }

            false -> if (rightClaw && leftClaw) {
                fullSequence()
            }
        }

        if (!(rightClaw && leftClaw)) {
            clawContainer = ClawContainer.Empty
            if (shoulderAngle() >= 90) {
                Lights.lightScenario = Lights.LightScenario.Released
                Rumble.RumbleController = Rumble.Released
            }
        }
    }
}
