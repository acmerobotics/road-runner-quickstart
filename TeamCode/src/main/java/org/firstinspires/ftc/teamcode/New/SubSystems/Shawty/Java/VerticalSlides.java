package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.PIDFcontroller;
import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.PIDParams;

public class VerticalSlides {

    public LeftSlide leftSlide;
    public RightSlide rightSlide;
    public static int resetValue = 0;

    public VerticalSlides(HardwareMap hardwareMap){
        leftSlide = new LeftSlide(hardwareMap);
        rightSlide = new RightSlide(hardwareMap);
    }

    public enum State{
        LOWERED,
        HANG,
        BASKET,
        BAR1,
        BAR2,
        IDLE
    }

    static State state = State.LOWERED;
    int target = 0;

    public void update(State slideState) {
        state = slideState;

        switch (state){
            //find height values, make code to test
            case LOWERED: target = 0; break;
            case BASKET: target = 1; break;
            case HANG: target = 2; break;
            case BAR1: target = 3; break;
            case BAR2: target = 4; break;
        }

        leftSlide.update(target);
        rightSlide.update(target);
    }

    public static class LeftSlide {

        DcMotor leftSlide;
        int prevPos;
        int leftEncoder;

        DigitalChannel leftDigitalChannel;
        LimitSwitch leftLimitSwitch = new LimitSwitch(leftDigitalChannel, resetValue);

        //tune PID coefficients
        PIDParams pidParams = new PIDParams(0.0,0.0,0.0,0.0);
        PIDFcontroller pidFcontroller = new PIDFcontroller(pidParams);

        public LeftSlide(HardwareMap hardwareMap){
            leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
            leftDigitalChannel = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        }

        public void update(int target) {
            leftEncoder += leftSlide.getCurrentPosition() - prevPos;
            prevPos = leftSlide.getCurrentPosition();

            leftLimitSwitch.update();
            if(leftLimitSwitch.state == LimitSwitch.State.RESET){
                leftEncoder = LimitSwitch.State.RESET.value;
            }

            double leftPower = pidFcontroller.calculate(target - leftEncoder);

            //idle mode for testing or emergency stop
            if(state == State.IDLE){
               leftPower = 0;
           }

            leftSlide.setPower(leftPower);
        }
    }

    public static class RightSlide {

        DcMotor rightSlide;
        int prevPos;
        int rightEncoder;

        DigitalChannel rightDigitalChannel;
        LimitSwitch rightLimitSwitch = new LimitSwitch(rightDigitalChannel, resetValue);

        //tune PID coefficients
        PIDParams pidParams = new PIDParams(0.0,0.0,0.0,0.0);
        PIDFcontroller pidFcontroller = new PIDFcontroller(pidParams);

        public RightSlide(HardwareMap hardwareMap){
            rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
            rightDigitalChannel = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        }

        public void update(int target) {
            rightEncoder += rightSlide.getCurrentPosition() - prevPos;
            prevPos = rightSlide.getCurrentPosition();

            rightLimitSwitch.update();
            if(rightLimitSwitch.state == LimitSwitch.State.RESET){
                rightEncoder = LimitSwitch.State.RESET.value;
            }

            double rightPower = pidFcontroller.calculate(target - rightEncoder);

            //idle mode for testing or emergency stop
            if(state == State.IDLE){
                rightPower = 0;
            }

            rightSlide.setPower(rightPower);
        }
    }
}
