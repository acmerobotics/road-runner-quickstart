package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.BacktrackingKt.Drive

class Localizer(hwMap: HardwareMap, startPose: Pose2d){

    var drive = Drive(hwMap, startPose)

    fun update() {
        drive.updatePoseEstimate()
    }

    val xPos: Double
        get() = drive.pose.position.y *-1
    val yPos: Double
        get() = drive.pose.position.x
    val heading: Rotation2d
        get() = drive.pose.heading

}
