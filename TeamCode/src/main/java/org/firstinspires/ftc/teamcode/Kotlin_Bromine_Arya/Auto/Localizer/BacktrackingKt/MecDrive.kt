package org.firstinspires.ftc.teamcode.BacktrackingKt

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RR.MecanumDrive
import org.firstinspires.ftc.teamcode.RR.messages.PoseMessage
import kotlin.math.abs

open class MecDrive(hardwareMap: HardwareMap, startPose: Pose2d) : MecanumDrive(hardwareMap, startPose) {

    lateinit var drive: Drive
    private var poseWithoutBacktracking: Pose2d = startPose

    fun createLocalizer(hwMap: HardwareMap){
        localizer = Localizer(hwMap, lazyImu.get(), PARAMS.inPerTick, drive)
    }

    override fun updatePoseEstimate(): PoseVelocity2d {
        val twist = localizer.update()
        pose += twist.value()

        poseWithoutBacktracking += twist.value()

        OverallError = doubleArrayOf(
            abs(poseWithoutBacktracking.position.x - pose.position.x),
            abs(poseWithoutBacktracking.position.y - pose.position.y),
            abs(poseWithoutBacktracking.heading - pose.heading)
        )

        poseHistory + pose
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(pose))

        return twist.velocity().value()
    }


    private var prevPose= DoubleArray(2)
    private var percentageCorrection=  DoubleArray(2)
    private var totalChange= arrayListOf(0.0,0.0)

    //This is returns the effectiveness of the Localizer in percentage
    fun totalMovement(): DoubleArray {
        val currentPose = doubleArrayOf(pose.position.x, pose.position.y)
        val difference = DoubleArray(2)
        for (i in 0..1) {
            difference[i] = abs(currentPose[i] - prevPose[i])
            totalChange[i] += difference[i]
            if (!totalChange.contains(0.0)) {
                percentageCorrection[i] = 100 * abs(OverallError[i]) / totalChange[i]
            }
        }
        prevPose = currentPose
        return percentageCorrection
    }
    companion object{
        var OverallError= DoubleArray(3)
    }
}
