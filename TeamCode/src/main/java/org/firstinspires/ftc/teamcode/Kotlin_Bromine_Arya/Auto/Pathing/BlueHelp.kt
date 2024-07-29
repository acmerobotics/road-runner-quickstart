package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Pathing

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Pathing.Paths.circle
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing.Sequencer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Localizer.Localizer
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Wait.runAsynchActionAfter

const val PI = Math.PI
class BlueHelp(hwMap: HardwareMap, localizer: Localizer, startPose: Pose2d) :
    Sequencer(hwMap, localizer) {

    private var DriveTo: Array<DoubleArray> = arrayOf(
        doubleArrayOf(
            startPose.position.x, startPose.position.y,
            startPose.heading.toDouble(), .5, .5, PI/15
        )
    )

    fun right() {

    }

    fun left() {
        when (MAJORCOMMAND) {
            0 -> DriveTo = circle
            1 -> manualGrab()
            2 -> {
                DriveTo = circle
                runAsynchActionAfter(600) { pixelDrop() }
            }
            3 -> {}
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

    }

    //ONLY FOR TUNING
    fun setPID(p: DoubleArray, i: DoubleArray, d:DoubleArray){
        drive.setPID(p,i,d)
    }

    private val camSide = PropLocation.Left

    // openCv = openCV(hwMap)
    fun updateCamSide() {
        //TODO(Constantly run while op mode is not active and send argument to Open CV file)
        //camside = openCV.getSide
    }

    fun Loop() {
        when (camSide) {
            PropLocation.Middle -> middle()
            PropLocation.Right -> right()
            PropLocation.Left -> left()
        }

        seekAndDrive(DriveTo)

        loop()
    }
}
