package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.PID_Components

import com.qualcomm.robotcore.util.ElapsedTime

//TODO(Move to A util class)


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

    fun setPIDF(kp: Double, ki: Double, kd: Double, kf:Double = 0.0) {
        this.kp = kp
        this.ki = ki
        this.kd = kd
        this.kf = kf
    }

    fun calculate(current: Double, target: Double): Double {
        error = target - current

        integral += (error * timer.seconds())

        derivitive = (error - beforeError) / timer.seconds()

        beforeError = error

        timer.reset()

        return derivitive * kd + integral * ki + error * kp
    }
}