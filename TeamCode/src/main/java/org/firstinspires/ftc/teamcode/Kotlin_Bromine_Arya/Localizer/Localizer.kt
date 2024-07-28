package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.BacktrackingKt.Drive

class Localizer(hwMap: HardwareMap, startPose: Pose2d): Localizers{

    var drive = Drive(hwMap, startPose)

    override fun update() {
        drive.updatePoseEstimate()
    }

    override val xPos: Double
        get() = drive.pose.position.y *-1
    override val yPos: Double
        get() = drive.pose.position.x
    override val heading: Rotation2d
        get() = drive.pose.heading

}

interface Localizers {

    fun update()

    val xPos: Double

    val yPos: Double

    val heading: Rotation2d

}