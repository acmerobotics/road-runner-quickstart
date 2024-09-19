package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.control.FeedforwardController;
import com.aimrobotics.aimlib.control.LowPassFilter;
import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.util.Mechanism;
import com.aimrobotics.aimlib.control.SimpleControlSystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

public class SlidesBase extends Mechanism {

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    String leftSlideName;
    String rightSlideName;

    double activeTargetPosition = 0;
    //TODO: Change position to put into the constructor. Committing to test and then sleeping
    PIDController pidController = new PIDController(0.1, 0.1, 0.1, 0.1, 0.1);
    FeedforwardController feedforwardController = new FeedforwardController(0.1, 0.1, 0.1, 0.1, 0.1);
    LowPassFilter lowPassFilter = new LowPassFilter(0.1);
    SimpleControlSystem controlSystem = new SimpleControlSystem(pidController, feedforwardController, lowPassFilter);

    public SlidesBase(String leftSlideName, String rightSlideName) {
        this.leftSlideName = leftSlideName;
        this.rightSlideName = rightSlideName;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); //TODO check if this is correct
    }

    public void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void update(double targetPosition) {

    }

    private double setSlidesTargetPosition(double targetPosition) {
        return 0.2; //TODO: Implement this
    }
}
