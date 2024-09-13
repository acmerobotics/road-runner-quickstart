package org.firstinspires.ftc.teamcode.New.SubSystems

interface SubSystems {
    fun update(){}
    val state: Any
        get() ={}
    fun update(gamepadInput: ArrayList<Float>){}
}