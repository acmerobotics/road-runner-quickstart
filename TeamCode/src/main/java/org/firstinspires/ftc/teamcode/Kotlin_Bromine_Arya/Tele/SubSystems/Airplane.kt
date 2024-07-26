package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Airplane(hwMap: HardwareMap) {

    private val launcher: Servo = hwMap.get(Servo::class.java, "air")

    init {
        launcher.scaleRange(0.0, .5)
        launcher.position = 0.0
    }

    fun launch() {
        launcher.position = 1.0
    }

}

