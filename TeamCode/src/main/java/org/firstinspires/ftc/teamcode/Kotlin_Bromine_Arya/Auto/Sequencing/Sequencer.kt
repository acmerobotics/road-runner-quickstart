package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Actions
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.PID_Components.PIDdrive
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer.Localizer
import kotlin.math.abs

open class Sequencer(hwMap: HardwareMap, private val localizer: Localizer) : Actions(hwMap) {
    val drive = PIDdrive(hwMap, localizer)

    companion object {
        var MAJORCOMMAND: Int = 0
        var hasReached: Boolean =false
    }
    //TODO()
    //start from somewhere differnt then {0,0,0}

    private fun hasReached(Target: DoubleArray): Boolean {
        return abs(localizer.yPos - Target[1]) <= Target[4] &&
                abs(localizer.xPos - Target[0]) <= Target[3]  &&
                abs(localizer.heading.toDouble()- Target[2]) <=Target[5]
    }

    var i = 0
    fun seekAndDrive(listOfPoints: Array<DoubleArray>) {
        drive.driveTo(listOfPoints[i])
        if (hasReached(listOfPoints[i])) {
            hasReached = true
            if (i == listOfPoints.size) {
                MAJORCOMMAND++
                i = 0
            }
            i++
        }
    }


}
