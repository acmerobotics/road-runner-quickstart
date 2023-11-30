package org.firstinspires.ftc.teamcode.testing.structureOptions.stupidInterfaceStructure;

import com.qualcomm.robotcore.hardware.DcMotor;

interface DT4 {
    Motors getMotors();


    default void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        getMotors().set(fl, fr, bl, br);
    }

    default void setDirections(DcMotor.Direction fl, DcMotor.Direction fr, DcMotor.Direction bl, DcMotor.Direction br){
        Motors motors = getMotors();
        motors.getFL().setDirection(fl);
        motors.getFR().setDirection(fr);
        motors.getBL().setDirection(bl);
        motors.getBR().setDirection(br);
    }

    default void setPowers(double fl, double fr, double bl, double br){
        double max = Math.max(1, Math.max(
                Math.max(fl, fr),
                Math.max(bl, br)
        ));
        Motors motors = getMotors();
        motors.getFL().setPower(fl/max);
        motors.getFR().setPower(fr/max);
        motors.getBL().setPower(bl/max);
        motors.getBR().setPower(br/max);
    }
}