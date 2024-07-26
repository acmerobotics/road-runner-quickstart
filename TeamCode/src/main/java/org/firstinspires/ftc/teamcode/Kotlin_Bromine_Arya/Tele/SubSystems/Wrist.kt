package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Wrist(hwMap: HardwareMap) : Shoulder(hwMap) {

    enum class WristPos {
        Raised,
        Lowered
    }

    val wrist: Servo = hwMap.get(Servo::class.java, "wrist")
    val wrist2: Servo = hwMap.get(Servo::class.java, "wrist2")

    companion object{
        var wristPos: WristPos = WristPos.Raised
    }

    fun wristAngle():Double{
        if (wristPos == WristPos.Lowered) {
            wrist2.position = .216
        } else {
            wrist2.position =shoulderAngle() /360
        }
        return shoulderAngle()/360
    }
    fun targetpos():Double{
        return wrist2.position
    }


}
