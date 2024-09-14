package org.firstinspires.ftc.teamcode.New.SubSystems.Kotlin

interface SubSystems {
    fun update(){}
    val state: Any
        get() ={}
    fun update(gamepadInput: ArrayList<Float>){}
}