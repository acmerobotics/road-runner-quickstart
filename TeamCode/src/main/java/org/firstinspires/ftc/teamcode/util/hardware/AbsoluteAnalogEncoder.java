package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AbsoluteAnalogEncoder {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;
    private double rotations = 0;
    private double previousAngle = 0;

    private double modifier;

    private final static double GEAR_RATIO = 3.0;


    public AbsoluteAnalogEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }

    private double gearRatio() {
        return ( ( 2 * Math.PI ) / GEAR_RATIO );
    }

//    public void updateRotations(Telemetry telemetry) {
//        double pos = getCurrentPositionOld();
//
//        telemetry.addData("______ previousAngle", previousAngle);
//        telemetry.addData("______ pos", pos);
//        telemetry.addData("______ gearRatio (rad)", gearRatio());
//
//        if (pos > previousAngle &&
//                  previousAngle < gearRatio() / 2 &&
//                  pos > gearRatio() / 2 &&
//                  pos - previousAngle > gearRatio() / 2) {
////                    telemetry.addLine("ADDING ROTATION ");
////                    telemetry.update();
//                    rotations -= 1;
//        } else if (pos < previousAngle &&
//                   previousAngle > gearRatio() / 2 &&
//                   pos < gearRatio() / 2 &&
//                   previousAngle - pos > gearRatio() / 2) {
//                    rotations += 1;
////                    telemetry.addLine("DELETING ROTATION ");
////                    telemetry.update();
//        }
//
//        previousAngle = pos;
//
//        telemetry.addData("______ Rotations ", rotations  );
//        telemetry.addData("______ Modifier ", (rotations ) * (Math.PI *2) );
//    }

    public void updateRotations(Telemetry telemetry) throws InterruptedException {
        double pos = getCurrentPositionOld();

        telemetry.addData("______ previousAngle", previousAngle);
        telemetry.addData("______ pos", pos);
        telemetry.addData("______ gearRatio (rad)", gearRatio());

        if (pos > previousAngle) {
                    telemetry.addLine("ADDING ROTATION ");
                    telemetry.update();
            rotations -= 1;
        } else if (pos < previousAngle) {
            rotations += 1;
                    telemetry.addLine("DELETING ROTATION ");
                    telemetry.update();

        }

        previousAngle = pos;

        telemetry.addData("______ Rotations ", rotations  );
        telemetry.addData("______ Modifier ", (rotations ) * (Math.PI *2) );
    }

    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }
    public AbsoluteAnalogEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }
    public boolean getDirection() {
        return inverted;
    }

    private double pastPosition = 0;

    private double updateCurrentRawReading() {
        double pos = Angle.norm((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange));
        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(Angle.normDelta(pastPosition)) > 0.1 || Math.abs(Angle.normDelta(pos)) < 1) {
            pastPosition = pos;
        }
        return pastPosition;
    }

    private double updateCurrentPosition() {
        double radianAngle = updateCurrentRawReading() * ((2 * Math.PI) / GEAR_RATIO);
//        double radianAngle = updateCurrentRawReading();
        return radianAngle;
    }

    public double getCurrentPosition() {
//        pastPosition = updateCurrentPosition() + (rotations * (2 * Math.PI));
        pastPosition = updateCurrentPosition() + ( ((rotations * 0)) * (2 * Math.PI) / GEAR_RATIO);
        return ((pastPosition));
    }

    public double getCurrentPositionOld() {
        double pos = Angle.norm((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI*2 - offset);
        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(Angle.normDelta(pastPosition)) > 0.1 || Math.abs(Angle.normDelta(pos)) < 1) pastPosition = pos;
        return ((pastPosition) );
    }

    public double positionModifier() {
        if (rotations == 0) {
            modifier = 0;
        } else if (rotations > 0) {
            modifier = (rotations ) * (Math.PI *2);
        } else if (rotations < 0) {

        }

        return getCurrentPosition() + modifier;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}