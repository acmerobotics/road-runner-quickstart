package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision extends Mechanism {

    private Limelight3A camera;
    private LLResult result;

    private int isRed; // 1 is true, 0 is false and blue
    public boolean isDataOld = false;

    private double[] outputVars;

    public Vision(int isRed) {
        this.isRed = isRed;
    }

    @Override
    public void init(HardwareMap hwMap) {
        camera = hwMap.get(Limelight3A.class, "cam");
    }


    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        updateReadings();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Tx", getTargetX(-999999));
        telemetry.addData("Ty", getTargetY(-999999));
        telemetry.addData("Ta", getTargetY(-999999));
        telemetry.update();
    }

    private void initCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }

    private void updateReadings() {
        camera.updatePythonInputs(new double[] {isRed, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = camera.getLatestResult();
        outputVars = result.getPythonOutput();
    }

    private boolean hasResult () {
        return result != null;
    }

    public void checkStale() {
        isDataOld =  result.getStaleness() >= 100;
    }

    public double getTargetX(double defaultVal) {
        if (hasResult()) {
            return defaultVal;
        }
        return outputVars[1];
    }

    public double getTargetY(double defaultVal) {
        if (result == null) {
            return defaultVal;
        }
        return outputVars[2];
    }

    public double getTargetAngle(double defaultVal) {
        if (result == null) {
            return defaultVal;
        }
        return outputVars[3];
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return result.getTa() > 2;
    }
}
