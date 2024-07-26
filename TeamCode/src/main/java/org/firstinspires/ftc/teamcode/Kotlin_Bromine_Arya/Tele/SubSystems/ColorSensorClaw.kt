package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.JavaUtil
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class ColorSensorClaw(hwMap: HardwareMap) {

    enum class Recognition {
        Left,
        Right,
        None,
    }

    companion object {
        private var colorSensorRecognition: Recognition = Recognition.None
    }

    private val colorSensor: ColorSensor =
        hwMap.get(ColorSensor::class.java, "colorSensor")
    private val colorSensor2: ColorSensor =
        hwMap.get(ColorSensor::class.java, "colorSensor2")

    fun checkForRecognition(clawChoice: Claw.ClawChoice = Claw.ClawChoice.Both): Boolean {
        //reset
        Claw.notBothClaws = false
        colorSensorRecognition = Recognition.None

        //right sensor
        (colorSensor as NormalizedColorSensor).gain = 2f
        val normalizedColors = (colorSensor as NormalizedColorSensor).normalizedColors
        val color = normalizedColors.toColor()
        val value = JavaUtil.colorToValue(color)

        //left sensor
        (colorSensor2 as NormalizedColorSensor).gain = 2f
        val normalizedColors2 = (colorSensor2 as NormalizedColorSensor).normalizedColors
        val color2 = normalizedColors2.toColor()
        val value2 = JavaUtil.colorToValue(color2)

        val left =
            ((colorSensor2 as DistanceSensor).getDistance(DistanceUnit.CM) < 5) && value2 >= .06
        val right =
            ((colorSensor as DistanceSensor).getDistance(DistanceUnit.CM) < 3) && value >= .06

        colorSensorRecognition =
            if (left) Recognition.Left else if (right) Recognition.Right else Recognition.None

        when (colorSensorRecognition) {
            Recognition.None -> {}
            Recognition.Left -> {
                Claw.clawChoice = Claw.ClawChoice.Left
                Claw.clawState = Claw.ClawState.CLOSE
                if (clawChoice != Claw.ClawChoice.Both) {
                    return true
                }
            }

            Recognition.Right -> {
                Claw.clawChoice = Claw.ClawChoice.Right
                Claw.clawState = Claw.ClawState.CLOSE
                if (clawChoice != Claw.ClawChoice.Both) {
                    return true
                }
            }
        }
        return false
    }
}














