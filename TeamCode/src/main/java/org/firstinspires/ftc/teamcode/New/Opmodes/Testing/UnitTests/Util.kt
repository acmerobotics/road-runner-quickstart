package org.firstinspires.ftc.teamcode.New.Opmodes.Testing.UnitTests

import org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin.Angle
import org.testng.annotations.Test
import kotlin.math.PI


class AngleWrap {

    @Test
    fun main(){
        val angle = PI *4 +.2
        println(Angle.wrap(angle))
    }

}
