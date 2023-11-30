package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;


public class DTEncoderState {
    private double fl;
    private double fr;
    private double bl;
    private double br;

    DTEncoderState(double fl, double fr, double bl, double br){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    double getFl(){
        return fl;
    }

    double getFr(){
        return fr;
    }

    double getBl(){
        return bl;
    }

    double getBr(){
        return br;
    }
}
