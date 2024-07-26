package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Paths

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Wait.runAsynchActionAfter

class BlueHelp (hwMap: HardwareMap):Sequencer(hwMap){

    private val PurplePixelLeft: Array<DoubleArray> = arrayOf(
        doubleArrayOf(7.0,17.0,0.0,3.0,2.0),
        doubleArrayOf(-3.0,0.0,0.0,3.0,2.0),
        doubleArrayOf(10.2,0.0,0.0,0.0,2.0)
    )
    private val toBoard_upperTruss: Array<DoubleArray> = arrayOf(
        doubleArrayOf(3.0,3.0,3.0),
        doubleArrayOf(3.0,3.0,3.0)
    )

    private val camSide = side.Middle
    // openCv = openCV(hwMap)
    fun updateCamSide(){
        //TODO(Constantly run while op mode is not active and send argument to Open CV file)
        //camside = openCV.getSide
    }

    fun Loop(){
        when(camSide){
            side.Middle-> middle()
            side.Right -> right()
            side.Left -> left()
        }
        loop()
    }

    fun right() {
        when (MAJORCOMMAND) {
            0 -> seekAndDrive(PurplePixelLeft)
            2 -> pixelDrop()
            3 -> seekAndDrive(toBoard_upperTruss)
            4 -> {seekAndDrive(PurplePixelLeft)
                runAsynchActionAfter(2000) {lowerArm()} }
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
            0 -> seekAndDrive(PurplePixelLeft)
            2 -> pixelDrop()
            3 -> seekAndDrive(toBoard_upperTruss)
            4 -> {}
            5 -> {}
            7 -> {}
            8 -> {}
            9 -> {}
            10 -> {}
            11 -> {}
        }
    }
    fun middle() {
        when (MAJORCOMMAND) {
            0 -> seekAndDrive(PurplePixelLeft)
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


    enum class side{
        Left,
        Right,
        Middle
    }
}
