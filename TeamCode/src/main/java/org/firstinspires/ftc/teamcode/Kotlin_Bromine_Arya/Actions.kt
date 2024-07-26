package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Airplane
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Claw
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.ColorSensorClaw
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.PurplePixel
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Shoulder
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems.Wrist

open class Actions(hardwareMap: HardwareMap) {

    private val claw = Claw(hardwareMap)
    private val airplane = Airplane(hardwareMap)
    private val purplePixel = PurplePixel(hardwareMap)
    private val shoulder = Shoulder(hardwareMap)
    private val wrist = Wrist(hardwareMap)
    private val colorSensor = ColorSensorClaw(hardwareMap)

    companion object{
        var isAuto: Boolean = false
    }


    //AUTOMATIC CHANGE IN MAJORCOMMAND
    fun autoGrabSpecific(clawChoice: Claw.ClawChoice) {
        Wrist.wristPos = Wrist.WristPos.Lowered
        Shoulder.shoulderPos = Shoulder.ShoulderPos.Lowered
        Claw.notBothClaws = colorSensor.checkForRecognition(clawChoice)
    }
    //AUTOMATIC CHANGE IN MAJORCOMMAND
    fun autoGrabBoth() {
        Wrist.wristPos = Wrist.WristPos.Lowered
        Shoulder.shoulderPos = Shoulder.ShoulderPos.Lowered
        Claw.notBothClaws = colorSensor.checkForRecognition()
        if(Claw.clawContainer == Claw.ClawContainer.Full){
            Claw.clawChoice = Claw.ClawChoice.Both
        }
    }

    fun lowerArm() {
        Shoulder.shoulderPos = Shoulder.ShoulderPos.Lowered
        Wrist.wristPos = Wrist.WristPos.Raised
    }

    fun raiseArm(shoulderLevel: Int) {
        Shoulder.shoulderLevel = shoulderLevel
        Wrist.wristPos = Wrist.WristPos.Raised
        Shoulder.shoulderPos = Shoulder.ShoulderPos.Raised
    }

    fun raiseArm() {
        Wrist.wristPos = Wrist.WristPos.Raised
        Shoulder.shoulderPos = Shoulder.ShoulderPos.Raised
    }

    fun release(clawChoice: Claw.ClawChoice) {
        Claw.clawChoice = clawChoice
        Claw.clawState = Claw.ClawState.OPEN
    }

    fun release() {
        Claw.clawState = Claw.ClawState.OPEN
    }

    //AUTOMATIC CHANGE IN MAJORCOMMAND
    fun manualGrab(clawChoice: Claw.ClawChoice) {
        Claw.clawChoice = clawChoice
        Claw.clawState = Claw.ClawState.CLOSE
        when (clawChoice) {
            Claw.ClawChoice.Both -> {
                if (Claw.clawContainer == Claw.ClawContainer.Full) {
                    Sequencer.MAJORCOMMAND++
                }
            }

            Claw.ClawChoice.Right, Claw.ClawChoice.Left -> if (Claw.clawContainer == Claw.ClawContainer.HalfFull) {
                Sequencer.MAJORCOMMAND++
            }
        }
    }
    fun manualGrab() {
        Claw.clawState = Claw.ClawState.CLOSE
        if(isAuto&& Claw.clawContainer == Claw.ClawContainer.Full){
            Sequencer.MAJORCOMMAND++
        }
    }

    //AUTOMATIC CHANGE IN MAJORCOMMAND
    fun pixelDrop() {
        if (purplePixel.drop() == PurplePixel.Status.Lowered) {
            //Wait.wait(300)
            Sequencer.MAJORCOMMAND++
        }
    }

    fun launchAirPlane() {
        airplane.launch()
    }

    fun loop() {
        claw.manual()
        claw.clawPos()
        wrist.wristAngle()
        shoulder.pidShoulder()
        shoulder.shoulderAngle()
    }

}