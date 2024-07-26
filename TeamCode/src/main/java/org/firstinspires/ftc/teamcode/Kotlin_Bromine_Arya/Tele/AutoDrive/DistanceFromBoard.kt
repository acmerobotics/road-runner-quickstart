package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.AutoDrive

import kotlin.math.sin

object DistanceFromBoard{
    private var boardOffset: Double = 1.0

    //Find the Attachments being considered
    fun distanceFromTAG(attachmentsList: Array<Array<Double>>): Double {
        var distance = 0.0
        for (i in attachmentsList){
            distance+= checkDistance(i)
        }
        return distance + boardOffset
    }

    private fun checkDistance(i: Array<Double>): Double {
        val mechanismLength: Double = i[0]
        val angle: Double = i[1]
        return mechanismLength * sin(Math.toRadians(angle)) / sin(Math.PI / 2)
    }

}
