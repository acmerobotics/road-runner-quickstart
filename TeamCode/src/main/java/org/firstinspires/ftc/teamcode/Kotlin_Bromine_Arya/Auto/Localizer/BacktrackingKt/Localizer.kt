package org.firstinspires.ftc.teamcode.BacktrackingKt

import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.BacktrackingKt.DualNumHelper
import org.firstinspires.ftc.teamcode.RR.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.RR.messages.ThreeDeadWheelInputsMessage

class Localizer(hardwareMap: HardwareMap, val imu: IMU, inPerTick: Double, private val drive: Drive) :
    ThreeDeadWheelLocalizer(hardwareMap, inPerTick) {

    private var par0PosDelta = 0
    private var par1PosDelta = 0
    private var perpPosDelta = 0
    private var deadWheelHeading= Rotation2d (0.0,0.0)
    private var heading = Rotation2d (0.0,0.0)
    private var listOfChanges= ArrayList<ArrayList<Double>>()
    private var listOfPoses= ArrayList<Pose2d>()
    var timer: ElapsedTime = ElapsedTime()
    private var listofEstimatedPosChanges= ArrayList<Twist2d>()
    init {
        timer.reset()
        imu.resetYaw()

        //reset odowheels (Have to do this because RR's custom encoder class does not let you Reset)
        val parx = hardwareMap.get(DcMotor::class.java, "par0")
        val pary = hardwareMap.get(DcMotor::class.java, "par1")
        parx.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        pary.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        parx.close()
        pary.close()
    }

    override fun update(): Twist2dDual<Time> {
        val readImu = timer.milliseconds().toInt() >= BacktrackingTUNING.Backtracking_TuningKt.timeBetween_Reads

        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()

        //get heading
        if (readImu) {
            val angles = imu.robotYawPitchRollAngles
            heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))
        }

        write(
            "THREE_DEAD_WHEEL_INPUTS",
            ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel)
        )

        if (!initialized) {
            initialized = true

            lastPar0Pos = par0PosVel.position
            lastPar1Pos = par1PosVel.position
            lastPerpPos = perpPosVel.position
            deadWheelHeading = Rotation2d.exp((par0PosVel.position - par1PosVel.position) / (PARAMS.par0YTicks - PARAMS.par1YTicks))

            return Twist2dDual(
                Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
            )
        }

        par0PosDelta = par0PosVel.position - lastPar0Pos
        par1PosDelta = par1PosVel.position - lastPar1Pos
        perpPosDelta = perpPosVel.position - lastPerpPos

        val deadWheelHeadingDelta =
            (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks)
        val axial =
            (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks)
        val lateral =
            (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta)

        val poseChange = arrayListOf(axial, lateral, deadWheelHeadingDelta)
        listOfChanges.add(poseChange)
        listOfPoses.add(drive.pose)

        val twist = Twist2dDual(
            Vector2dDual(
                DualNumHelper.createDualNum(
                    doubleArrayOf(
                        axial,
                        (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks)
                    )
                ).times(inPerTick),
                DualNumHelper.createDualNum(
                    doubleArrayOf(
                        lateral,
                        (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity)
                    )
                ).times(inPerTick)
            ),
            DualNumHelper.createDualNum(
                doubleArrayOf(
                    deadWheelHeadingDelta,
                    (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                )
            )
        )

        listofEstimatedPosChanges.add(twist.value())
        //TODO(CHange project to fix this)

        deadWheelHeading = Rotation2d.exp((par0PosVel.position - par1PosVel.position) / (PARAMS.par0YTicks - PARAMS.par1YTicks))

        if (readImu) {
            val headingDrift = heading - deadWheelHeading
            drive.correctCurrentPose(listOfChanges, listOfPoses, listofEstimatedPosChanges,headingDrift)
            timer.reset()
        }

        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
        lastPerpPos = perpPosVel.position


        return twist
    }

}
