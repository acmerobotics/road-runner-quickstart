package org.firstinspires.ftc.teamcode.testing.structureOptions.inheritanceStructure;

public class MecDT extends DT{
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
    void directionalPow(double forward, double side, double rot){
        double maxTrans = Math.max(1, Math.max(forward, side));
        double maxRot = Math.max(1, rot);
        motorPow(
                (forward - side)/maxTrans*transScl/Math.sqrt(2) - rot/maxRot*rotScl,
                (forward + side)/maxTrans*transScl/Math.sqrt(2) + rot/maxRot*rotScl,
                (forward + side)/maxTrans*transScl/Math.sqrt(2) - rot/maxRot*rotScl,
                (forward - side)/maxTrans*transScl/Math.sqrt(2) + rot/maxRot*rotScl
        );
    }
    void absoluteDirectionalPow(double robotDir, double forward, double side, double rot){
        // Visualization of why this works: https://www.desmos.com/calculator/kgy249h8rj
        directionalPow(
                forward*Math.cos(robotDir) + side*Math.sin(robotDir),
                forward*Math.cos(robotDir + Math.PI/2) + side*Math.sin(robotDir + Math.PI/2),
                rot
        );
    }
}
