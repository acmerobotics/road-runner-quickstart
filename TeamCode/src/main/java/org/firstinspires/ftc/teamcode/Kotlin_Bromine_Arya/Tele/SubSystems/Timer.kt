package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.util.ElapsedTime

object Timer {
    //goes in teleop
    //val hasMoved = (Math.abs(gamepad1.right_stick_x.x)>0 || Math.abs(gamepad1.left_stick_x)>0 || Math.abs(gamepad1.left_stick_y)>0)

    private val clock: ElapsedTime = ElapsedTime()
    private var hasDoneIt: Boolean = false
        fun startClock(hasStarted: Boolean){
            if(hasStarted && !hasDoneIt){
                clock.reset()
                hasDoneIt = true
            }
            if (clock.seconds() >= 60) {
                Rumble.RumbleController = Rumble.Endgame
                Lights.lightScenario = Lights.LightScenario.Endgame
            }
        }



}