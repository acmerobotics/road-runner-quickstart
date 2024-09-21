package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin

import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.New.Opmodes.Testing.Opmodes.Localizer.HeadingTuner
import org.firstinspires.ftc.teamcode.New.RRlocalizer.ThreeDeadWheelLocalizer
import kotlin.math.abs

class TeleLocalizer(hardwareMap: HardwareMap) {

    private var PARAMS = ThreeDeadWheelLocalizer.Params()

    private var par0: Encoder
    private var par1: Encoder

    private val imu: IMU = hardwareMap.get(IMU::class.java, "imu")
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
        par0 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "par0")))
        par1 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "par1")))
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
    private var currentHeading = 0.0
    private var deadWheelHeading = 0.0
    private var lastDrift = 0.0
    private var thetaError = 0.0
    private var rightHalfTurns= 0
    private var lastHeading = 0.0

    fun updateHeading() {
        val readImu = timer.milliseconds().toInt() >= HeadingTuner.timeBetweenRead

        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()

        //get heading
        if (readImu) {
            val angles = imu.robotYawPitchRollAngles
            heading = angles.getYaw(AngleUnit.RADIANS)
            //Has coeffecients for both and right

            if(rightHalfTurns >0) {
                val unWrappedHeading = (rightHalfTurns * Math.PI) + (Math.PI - abs(heading))
                val coApplied = unWrappedHeading * HeadingTuner.Rcoeffecient
                heading += (unWrappedHeading-coApplied)
            }
            else if(rightHalfTurns<0){
                val unWrappedHeading = abs((rightHalfTurns * Math.PI) - (Math.PI - abs(heading)))
                val coApplied = unWrappedHeading * HeadingTuner.Lcoeffecient
                heading += (unWrappedHeading-coApplied)
            }
        }

        //This happens once
        if (!initialized) {
            initialized = true

            lastPar0Pos = par0PosVel.position
            lastPar1Pos = par1PosVel.position
            deadWheelHeading =
                ((par0PosVel.position - offsetPar0) - (par1PosVel.position - offsetPar1)) / (PARAMS.par0YTicks - PARAMS.par1YTicks)
            currentHeading = deadWheelHeading
        }

        deadWheelHeading =
            ((par0PosVel.position - offsetPar0) - par1PosVel.position - offsetPar1) / (PARAMS.par0YTicks - PARAMS.par1YTicks)

        if (readImu) {
            val headingDrift = Angle.wrap(heading - deadWheelHeading)
            thetaError += (lastDrift - headingDrift)
            lastDrift = headingDrift
            timer.reset()
        }

        currentHeading = Angle.wrap(deadWheelHeading - thetaError)

        checkForRot()

        lastHeading = currentHeading
        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
    }

    fun getRotation(): Double {
        return currentHeading
    }

    private fun checkForRot(){
        if(lastHeading>Math.PI/2 && currentHeading < -Math.PI/2){
            rightHalfTurns++
        }
        else if(lastHeading< -Math.PI/2 && currentHeading > Math.PI/2){
            rightHalfTurns--
        }
    }
}