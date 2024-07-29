package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Actions
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Angle
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.PID_Components.PIDdrive
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.Localizer
import kotlin.math.abs

open class Sequencer(hwMap: HardwareMap, private val localizer: Localizer) : Actions(hwMap) {
    val drive = PIDdrive(hwMap, localizer)

    companion object {
        var MAJORCOMMAND: Int = 0
    }
    //TODO()
    //start from somewhere differnt then {0,0,0}

    private fun hasReached(target: DoubleArray): Boolean {
        return abs(localizer.yPos - target[1]) <= target[4] &&
                abs(localizer.xPos - target[0]) <= target[3] &&
                abs(Angle.wrap(localizer.heading.toDouble() - target[2])) <= target[5]
    }

    var i = 0
    private var hasReached = false
    private var current = 0
    private var previous = 0
    private var timesChanged = 0
    fun seekAndDrive(listOfPoints: Array<DoubleArray>) {
        current = MAJORCOMMAND

        //If moving to last Point in sequence
        if (i == listOfPoints.size - 1) {
            //if reached point
            if (hasReached(listOfPoints[i]) && !hasReached) {
                MAJORCOMMAND++
                hasReached = true
            }
            if (current != previous) {
                timesChanged++
            }
            //3 because u have to account for has reached change in this function
            if (timesChanged == 2) {
                i = 0
                hasReached = false
                timesChanged = 0
            } else drive.driveTo(listOfPoints[listOfPoints.size - 1])
        } else {
            drive.driveTo(listOfPoints[i])
            if (hasReached(listOfPoints[i])) {
                i++
            }
        }
        previous = current
    }
}
