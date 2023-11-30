package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

public class MecDT extends DT {
    private double transScl = 1;
    private double rotScl = 1;


    void setScls(double trans, double rot){
        transScl = trans;
        rotScl = rot;
    }
    void setTransScl(double scl){
        transScl = scl;
    }
    void setRotScl(double scl){
        rotScl = scl;
    }

    Movement calcPosition(){
        DTEncoderState state = createEncoderState();
        return new Movement(
                state.getFl() + state.getFr() + state.getBl() + state.getBr(),
                state.getFl() - state.getFr() - state.getBl() + state.getBr(),
                -state.getFl() + state.getFr() + -state.getBl() + state.getBr()
        );
    }

    void mecPow(double x, double y, double rot){
        double maxTrans = Math.max(1, Math.max(y, x));
        double maxRot = Math.max(1, rot);
        motorPow(
                (y + x)/maxTrans/Math.sqrt(2)*transScl - rot/maxRot*rotScl,
                (y - x)/maxTrans/Math.sqrt(2)*transScl + rot/maxRot*rotScl,
                (y - x)/maxTrans/Math.sqrt(2)*transScl - rot/maxRot*rotScl,
                (y + x)/maxTrans/Math.sqrt(2)*transScl + rot/maxRot*rotScl
        );
    }
    void absMecPow(double robotDir, double x, double y, double rot){
        // Visualization of why this works: https://www.desmos.com/calculator/kgy249h8rj
        mecPow(
                y*Math.cos(robotDir) + x*Math.sin(robotDir),
                y*Math.cos(robotDir + Math.PI/2) + x*Math.sin(robotDir + Math.PI/2),
                rot
        );
    }
}
