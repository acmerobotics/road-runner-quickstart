package org.firstinspires.ftc.teamcode.testing.structureOptions.stupidInterfaceStructure;

public class Drivetrain implements Mecanum {
    private Motors motors = new Motors();

    public Motors getMotors(){
        return motors;
    }


    private Position position = new Position();
    public Position getPosition(){
        return position;
    }
}
