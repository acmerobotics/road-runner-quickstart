package org.firstinspires.ftc.teamcode.New.SubSystems.Java;

public interface JavaSubsystems {
    void update();
    default void updateMotorEncoder(int encoder){}
}
