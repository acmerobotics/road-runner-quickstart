package org.firstinspires.ftc.teamcode.New.SubSystems

import org.junit.Assert.*
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.*

import org.junit.Test

class AngleTest {

    @Test
    fun wrap() {
        val angle = Angle
        val incr = 0.01
        val epsilon = 0.0

        //Test values that SHOULD NOT wrap
        var expected = -Math.PI
        while(expected <= Math.PI){
            val actual = angle.wrap(expected)

            assertEquals(expected, actual, epsilon)

            expected += incr
        }

        //Test that values > Math.PI wrap
        var testValue = Math.PI + 1
        expected = testValue - Math.PI * 2
        var actual = angle.wrap(testValue)
        assertEquals(expected, actual, epsilon)

        //Test that values < -Math.PI wrap
        testValue = -Math.PI - 1
        expected = testValue + Math.PI * 2
        actual = angle.wrap(testValue)
        assertEquals(expected, actual, epsilon)
    }
}