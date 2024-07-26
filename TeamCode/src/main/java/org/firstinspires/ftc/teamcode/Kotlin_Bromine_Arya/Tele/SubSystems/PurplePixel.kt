package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class PurplePixel(hwMap: HardwareMap) {

    enum class Status {
        Raised,
        Lowered
    }


    private var pixelDropper: Servo = hwMap.get(Servo::class.java, "servo")
    var status: Status = Status.Raised

    init {
        pixelDropper.position = 0.0
    }

    fun drop(): Status {
        pixelDropper.position = 1.0
        return if (pixelDropper.position >= .8) {
            Status.Raised
        } else {
            Status.Lowered
        }
    }
}


