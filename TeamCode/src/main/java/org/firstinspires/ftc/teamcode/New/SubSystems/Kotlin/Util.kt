package org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.PI



object Angle {
    fun wrap(theta: Double): Double {
        require(theta in -2*PI..2*PI)
            var angle = theta
            while (angle > PI) angle -= PI*2
            while (angle < -PI) angle += PI*2
            return angle
    }
}

data class PIDParams(val kp: Double, val ki: Double, val kd: Double, val kf: Double =0.0)

data class PIDFcontroller(var params: PIDParams) {

    private var integral = 0.0
    private val timer: ElapsedTime = ElapsedTime()
    private var previousError = 0.0
    fun calculate(error: Double): Double {

        integral += (error * timer.seconds())
        val derivative = (error-previousError )/ timer.seconds()
        timer.reset()

        //add FF
        return (derivative * params.kd + integral * params.ki + error * params.kp).coerceIn(-1.0,1.0)
    }
}


object Wait {
    val wait: ElapsedTime = ElapsedTime()
    private var runOnce = false

//    fun waitFor(timeInMs: Int) {
//        if (timeStamp(timeInMs)) {
//            Sequencer.MAJORCOMMAND++
//        }
//    }

    fun <Parameter> runAsynchActionAfter(
        timeInMs: Int,
        action: (Parameter) -> Unit,
        parameter: Parameter
    ) {
        if (timeStamp(timeInMs)) action(parameter)
    }

    fun runAsynchActionAfter(timeInMs: Int, action: () -> Unit) {
        if (timeStamp(timeInMs)) action()
    }

    private fun timeStamp(timeInMs: Int): Boolean {
        var timeStamp = 0.0
        if (!runOnce) {
            timeStamp = wait.milliseconds()
        }
        runOnce = true

        return if (timeInMs <= wait.milliseconds() - timeStamp) {
            runOnce = false
            true
        } else false
    }

}
