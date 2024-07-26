package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern

object Lights {
    var lightScenario: LightScenario = LightScenario.Released

    fun getColor(lightScenario: LightScenario): BlinkinPattern {
        return when(lightScenario){
            LightScenario.ClawFull -> BlinkinPattern.GREEN
            LightScenario.Released -> BlinkinPattern.RED
            LightScenario.ClawSemiFull -> BlinkinPattern.BLUE
            LightScenario.Endgame -> BlinkinPattern.RAINBOW_LAVA_PALETTE
        }
    }

    enum class LightScenario {
        ClawSemiFull,
        Released,
        ClawFull,
        Endgame
    }
}
