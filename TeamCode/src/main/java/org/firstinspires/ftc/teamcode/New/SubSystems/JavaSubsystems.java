package org.firstinspires.ftc.teamcode.New.SubSystems;

public interface JavaSubsystems {
    void update();
    default void updateMotorEncoder(int encoder){}
}
