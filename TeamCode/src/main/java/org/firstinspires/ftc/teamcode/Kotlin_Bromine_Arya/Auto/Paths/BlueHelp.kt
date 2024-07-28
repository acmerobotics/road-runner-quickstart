package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Paths

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Wait.runAsynchActionAfter
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer.Localizer

const val PI = Math.PI
class BlueHelp(hwMap: HardwareMap, localizer: Localizer, startPose: Pose2d) :
    Sequencer(hwMap, localizer) {

    //TODO(Add heading)

    private var DriveTo: Array<DoubleArray> = arrayOf(
        doubleArrayOf(
            startPose.position.x, startPose.position.y,
            0.0, .5, .5
        )
    )
    private val purplePixelLeft: Array<DoubleArray> = arrayOf(
        doubleArrayOf(-7.0, 16.0, 0.0, 1.5, 1.5, PI/8),
        doubleArrayOf(0.0, 2.0, PI/2, 1.5, 1.5, PI/8),
        doubleArrayOf(-7.0, 16.0, 0.0, 1.5, 1.5, PI/8),
        doubleArrayOf(0.0, 2.0, PI/2, 1.5, 1.5, PI/8),
        doubleArrayOf(-7.0, 16.0, 0.0, 1.5, 1.5, PI/8),
        doubleArrayOf(0.0, 2.0, PI/2, 1.5, 1.5, PI/8)
    )
    private val toBoardupperTruss: Array<DoubleArray> = arrayOf(
        doubleArrayOf(3.0, 3.0, 3.0),
        doubleArrayOf(3.0, 3.0, 3.0)
    )

    private val camSide = side.Middle

    // openCv = openCV(hwMap)
    fun updateCamSide() {
        //TODO(Constantly run while op mode is not active and send argument to Open CV file)
        //camside = openCV.getSide
    }

    fun Loop() {
        when (camSide) {
            side.Middle -> middle()
            side.Right -> right()
            side.Left -> left()
        }
        seekAndDrive(DriveTo)


        //loop()
    }

    fun right() {
        when (MAJORCOMMAND) {
            0 -> seekAndDrive(purplePixelLeft)
            2 -> pixelDrop()
            3 -> seekAndDrive(toBoardupperTruss)
            4 -> {
                seekAndDrive(purplePixelLeft)
                runAsynchActionAfter(2000) { lowerArm() }
            }

            5 -> {}
            7 -> {}
            8 -> {}
            9 -> {}
            10 -> {}
            11 -> {}
        }
    }

    fun left() {
        when (MAJORCOMMAND) {
            0 -> seekAndDrive(purplePixelLeft)
            2 -> pixelDrop()
            3 -> seekAndDrive(toBoardupperTruss)
            4 -> {}
            5 -> {}
            7 -> {}
            8 -> {}
            9 -> {}
            10 -> {}
            11 -> {}
        }
    }

    //TODO(Maybe add operational override that instead of adding to list adds to variables for drive target)
    fun middle() {
        when (MAJORCOMMAND) {
            0 -> DriveTo = purplePixelLeft
            1 -> {}
            2 -> {}
            3 -> {}
            5 -> {}
            7 -> {}
            8 -> {}
            9 -> {}
            10 -> {}
            11 -> {}
        }
    }


    enum class side {
        Left,
        Right,
        Middle
    }

    fun setPID(p: DoubleArray, i: DoubleArray, d:DoubleArray){
        drive.setPID(p,i,d)
    }

}
