package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer

import org.firstinspires.ftc.teamcode.BacktrackingKt.Drive

object LOCALIZER {
    lateinit var mecDrive: Drive
    val xPos = mecDrive.pose.position.y
    val yPos = mecDrive.pose.position.y
    val headingPos = mecDrive.pose.heading.toDouble()
}
