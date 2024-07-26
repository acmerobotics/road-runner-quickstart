package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect

object Rumble {
    val Grabbed: RumbleEffect = RumbleEffect.Builder()
        .addStep(.75, .75, 1200)
        .build()
    val Released: RumbleEffect = RumbleEffect.Builder()
        .addStep(.5, .5, 1300)
        .build()
    val Endgame: RumbleEffect = RumbleEffect.Builder()
        .addStep(.4, .4, 1000)
        .build()
    private val Start: RumbleEffect = RumbleEffect.Builder()
        .addStep(.2, .2, 1000)
        .build()
    var RumbleController: RumbleEffect? = Start
    //Consistently have teleop run Rumble Controller
}
