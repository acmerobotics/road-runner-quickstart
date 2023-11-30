package org.firstinspires.ftc.teamcode.testing.structureOptions.stupidInterfaceStructure;

import com.qualcomm.robotcore.hardware.DcMotor;

class Motors{
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    void set(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    DcMotor getFL(){
        return fl;
    }
    DcMotor getFR(){
        return fr;
    }
    DcMotor getBL(){
        return bl;
    }
    DcMotor getBR(){
        return br;
    }

}