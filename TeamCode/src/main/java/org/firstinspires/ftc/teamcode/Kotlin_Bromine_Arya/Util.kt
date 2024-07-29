package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import kotlin.math.PI

private const val double = Math.PI * 2

object Angle {
    fun wrap(theta: Double): Double {
            var angle = theta
            while (angle > PI) angle -= double
            while (angle < -PI) angle += double
            return angle
    }
}


//TODO(Once Tuning is done add constructure to class so pid values are set)
class PIDController {
    private val timer: ElapsedTime = ElapsedTime()
    private var kp: Double = 0.0
    private var ki: Double = 0.0
    private var kd: Double = 0.0
    private var kf: Double = 0.0
    private var integral: Double = 0.0
    private var beforeError: Double = 0.0
    private var derivitive: Double = 0.0
    private var error: Double = 0.0

    fun setPIDF(kp: Double, ki: Double, kd: Double, kf: Double = 0.0) {
        this.kp = kp
        this.ki = ki
        this.kd = kd
        this.kf = kf
    }

    fun calculate(error: Double): Double {
        integral += (error * timer.seconds())

        derivitive = (error - beforeError) / timer.seconds()

        beforeError = error

        timer.reset()

        return derivitive * kd + integral * ki + error * kp
    }
}

object Wait {
    val wait: ElapsedTime = ElapsedTime()
    private var runOnce = false

    fun waitFor(timeInMs: Int) {
        if (timeStamp(timeInMs)) {
            Sequencer.MAJORCOMMAND++
        }
    }

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
