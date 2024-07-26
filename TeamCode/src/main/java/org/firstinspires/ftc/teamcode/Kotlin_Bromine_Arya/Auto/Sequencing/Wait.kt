package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing

import com.qualcomm.robotcore.util.ElapsedTime

object Wait {
    val wait: ElapsedTime = ElapsedTime()

    fun waitInPlaceFor(timeInMs: Int) {
        wait.reset()
        if (timeInMs < wait.milliseconds()) {Sequencer.MAJORCOMMAND++ }
    }
    fun waitFor(timeInMs: Int) {
        wait.reset()
        if (timeInMs <= wait.milliseconds()) {
            Sequencer.MAJORCOMMAND++}
    }
    fun <Paramter> runAsynchActionAfter(timeInMs: Int, action: (Paramter) -> Unit, parameter: Paramter){
        wait.reset()
        if (timeInMs <= wait.milliseconds()) {action(parameter)}
    }
    fun runAsynchActionAfter(timeInMs: Int, action: ()-> Unit){
        wait.reset()
        if (timeInMs <= wait.milliseconds()) {
            action()
        }
    }

}
