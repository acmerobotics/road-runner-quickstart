package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Tele.SubSystems

import com.acmerobotics.roadrunner.ftc.Encoder
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.BacktrackingKt.BacktrackingTUNING
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Angle
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.`Testing$Tuning`.`Subsystems$Tele`.LoopTimes.HeadingTuner
import org.firstinspires.ftc.teamcode.RR.ThreeDeadWheelLocalizer

class TeleLocalizer(hardwareMap: HardwareMap) {

    var PARAMS: ThreeDeadWheelLocalizer.Params = ThreeDeadWheelLocalizer.Params()

    lateinit var par0: Encoder
    lateinit var par1: Encoder

    val imu: IMU = hardwareMap.get(IMU::class.java, "imu")
    val timer = ElapsedTime()

    init {
        //reset odowheels (Have to do this because RR's custom encoder class does not let you Reset)
        val parx = hardwareMap.get(DcMotor::class.java, "par0")
        val pary = hardwareMap.get(DcMotor::class.java, "par1")
        parx.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        pary.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        parx.close()
        pary.close()

        val orientationOnRobot = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()
        timer.reset()
    }

    private var par1Pos = 0
    private var par0Pos = 0
    private var offsetPar1 = 0
    private var offsetPar0 = 0
    fun resetHeading() {
        //TODO(MAKE SURE ONLY RAN ONCE (Gamepad Should not have double inputs))
        imu.resetYaw()
        offsetPar1 += par1Pos
        offsetPar0 += par0Pos
    }

    var initialized = false
    private var lastPar0Pos: Int = 0
    private var lastPar1Pos: Int = 0
    var heading = 0.0
    private var finalHeading = 0.0
    private var deadWheelHeading = 0.0
    private var lastDrift = 0.0
    private var thetaError = 0.0
    private var rotationcount= 0.0

    fun updateHeading() {
        val readImu = timer.milliseconds().toInt() >= HeadingTuner.TeleLocalizer.timeBetweenRead

        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()

        //get heading
        if (readImu) {
            val angles = imu.robotYawPitchRollAngles
            heading = angles.getYaw(AngleUnit.RADIANS)
            //TODO(ADD TUNING COEFFECIENT)
        }

        //This happens once
        if (!initialized) {
            initialized = true

            lastPar0Pos = par0PosVel.position
            lastPar1Pos = par1PosVel.position
            deadWheelHeading =
                ((par0PosVel.position - offsetPar0) - (par1PosVel.position - offsetPar1)) / (PARAMS.par0YTicks - PARAMS.par1YTicks)
            finalHeading = deadWheelHeading
        }

        deadWheelHeading =
            ((par0PosVel.position - offsetPar0) - par1PosVel.position - offsetPar1) / (PARAMS.par0YTicks - PARAMS.par1YTicks)

        if (readImu) {
            val PreviousDeadWheelHeading = deadWheelHeading
            val headingDrift = Angle.wrap(heading - deadWheelHeading)
            thetaError += (lastDrift - headingDrift)
            lastDrift = headingDrift
            timer.reset()


            //im lazy and don't wanna figure out how to get pi in radians rn
            if ((PreviousDeadWheelHeading > Math.PI/2) && deadWheelHeading > Math.PI/2);||
                (PreviousDeadWheelHeading <-Math.PI/2 && deadWheelHeading < Math.PI/2);
            ++rotationcount

            }

        }

        finalHeading = Angle.wrap(deadWheelHeading - thetaError)

        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
    }

    fun getRotation(): Double {
        return finalHeading
    }

}