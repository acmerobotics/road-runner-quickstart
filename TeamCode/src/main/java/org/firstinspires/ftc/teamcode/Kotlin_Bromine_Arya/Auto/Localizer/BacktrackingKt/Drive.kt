package org.firstinspires.ftc.teamcode.BacktrackingKt

import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2dDual
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.BacktrackingKt.DualNumHelper

class Drive(hwMap: HardwareMap, startPos: Pose2d) : MecDrive(hwMap, startPos) {

    private var lastDrift = 0.0

    init {
        this.drive = this
        createLocalizer(hwMap)
    }

    companion object {
        var angleError = 0.0
    }

    fun correctCurrentPose(
        changesInPos: ArrayList<ArrayList<Double>>,
        posHistory: ArrayList<Pose2d>,
        twistChanges: ArrayList<Twist2d>,
        totalAngleDrift: Double
    ) {
        angleError += (totalAngleDrift - lastDrift)

        //loop through each estimated pose
        for (i in 0..<changesInPos.size) {
            val timeDrift = 1 / changesInPos.size * totalAngleDrift
            correctThePose(
                doubleArrayOf(
                    changesInPos[i][0],
                    changesInPos[i][1],
                    changesInPos[i][2] + timeDrift
                ),
                posHistory[i],
                twistChanges[i]
            )
        }

        changesInPos.clear()
        posHistory.clear()
        twistChanges.clear()

        lastDrift = angleError
    }

    private var lastErrors = DoubleArray(3)
    private var errors = DoubleArray(2)

    private fun correctThePose(change: DoubleArray, prevPose: Pose2d, estTwist: Twist2d) {
        val dualNums = ArrayList<DualNum<Time>>()
        for (i in 0..2) {
            dualNums.add(
                DualNumHelper.createDualNum(
                    doubleArrayOf(
                        change[i],
                        0.0,
                    )
                )
            )
        }
        val imuTwist = Twist2dDual(
            Vector2dDual(
                dualNums[0].times(PARAMS.inPerTick),
                dualNums[1].times(PARAMS.inPerTick)
            ),
            dualNums[2].times(PARAMS.inPerTick)
        )

        //because RR does not support subtracting twists or poses

        //Find pose using odo twist
        var previousPose = prevPose
        previousPose += estTwist
        val prevPosePoses = doubleArrayOf(previousPose.position.x, previousPose.position.y)

        //Find pose using imu twist
        var newPose = prevPose
        newPose += imuTwist.value()
        val newPosePoses = doubleArrayOf(newPose.position.x, newPose.position.y)

        for (i in 0..1) {
            errors[i] += newPosePoses[i] - prevPosePoses[i]
        }

        pose = Pose2d(
            pose.position.x + errors[0] - lastErrors[0],
            pose.position.y + errors[1] - lastErrors[1],
            (pose.heading + (angleError - lastErrors[2])).toDouble()
        )

        lastErrors = doubleArrayOf(
            errors[0],
            errors[1],
            angleError
        )
    }
}
