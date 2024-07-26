package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.Sequencing

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Actions
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Auto.PID_Components.PIDdrive
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Localizer.LOCALIZER

open class Sequencer(private val hwMap: HardwareMap): Actions(hwMap) {
    val drive = PIDdrive(hwMap)
    companion object{
        var MAJORCOMMAND: Int = 0
    }

    private var singlePoint: Int = 0
    //TODO()
    //start from somewhere differnt then {0,0,0}

    private fun hasReached(Target: DoubleArray): Boolean {
        return Math.abs(LOCALIZER.yPos - Target[1]) <= Target[4] && Math.abs(LOCALIZER.xPos- Target[0])<= Target[3]
        //make it optional so that there is a default TODO()
        // Math.abs(LOCALIZER.Heading()- target[MinorCommand][2])<=target[MinorCommand][5]
    }
    fun seekAndDrive(listOfPoints: Array<DoubleArray>) {
        if (listOfPoints.size == 1) {
            drive.driveTo(listOfPoints[0])
            if (hasReached(listOfPoints[0])) {
                MAJORCOMMAND++
                singlePoint = 0
            }
        }
        else{
            if (hasReached(listOfPoints[singlePoint])) {
                singlePoint++
                if(singlePoint == listOfPoints.size){
                    MAJORCOMMAND++
                    singlePoint++ }
            }
            if(singlePoint < listOfPoints.size){drive.driveTo(listOfPoints[singlePoint]) }
        }


    }


}
