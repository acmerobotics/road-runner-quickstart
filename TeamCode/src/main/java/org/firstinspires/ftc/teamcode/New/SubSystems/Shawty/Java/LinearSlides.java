package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.PIDFcontroller;
import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.PIDParams;

public class LinearSlides {

    public LeftSlide leftSlide;
    public RightSlide rightSlide;
    public int resetValue = 0;

    public LinearSlides(HardwareMap hardwareMap){
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

    State state = State.IDLE;
    int target = 0;

    public void update() {

        switch (state){
            //find height values, make code to test
            case LOWERED: target = 0; break;
            case BASKET: target = 1; break;
            case HANG: target = 2; break;
            case BAR1: target = 3; break;
            case BAR2: target = 4; break;
        }

        leftSlide.update(target, state);
        rightSlide.update(target, state);
    }

    public class LeftSlide {

        DcMotor leftSlide;
        int leftEncoder;

        DigitalChannel leftDigitalChannel;

        //tune PID coefficients
        PIDParams pidParams = new PIDParams(0.0,0.0,0.0,0.0);
        PIDFcontroller pidFcontroller = new PIDFcontroller(pidParams);

        public LeftSlide(HardwareMap hardwareMap){
            leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
            leftDigitalChannel = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        }

        LimitSwitch leftLimitSwitch = new LimitSwitch(leftDigitalChannel, resetValue);
        int prevPos = leftSlide.getCurrentPosition();

        public void update(int target, State state) {
            leftEncoder += leftSlide.getCurrentPosition() - prevPos;
            prevPos = leftSlide.getCurrentPosition();

            leftLimitSwitch.update();
            if(leftLimitSwitch.state == LimitSwitch.State.PRESSED){
                leftEncoder = leftLimitSwitch.resetValue;
            }

            double leftPower = pidFcontroller.calculate(target - leftEncoder);

            //idle mode for testing or emergency stop
            if(state == State.IDLE){
               leftPower = 0;
           }

            leftSlide.setPower(leftPower);
        }
    }

    public class RightSlide {
        DcMotor rightSlide;
        int rightEncoder;

        DigitalChannel rightDigitalChannel;

        //tune PID coefficients
        PIDParams pidParams = new PIDParams(0.0,0.0,0.0,0.0);
        PIDFcontroller pidFcontroller = new PIDFcontroller(pidParams);

        public RightSlide(HardwareMap hardwareMap){
            rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
            rightDigitalChannel = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        }

        LimitSwitch rightLimitSwitch = new LimitSwitch(rightDigitalChannel, resetValue);
        int prevPos = rightSlide.getCurrentPosition();

        public void update(int target, State state) {
            rightEncoder += rightSlide.getCurrentPosition() - prevPos;
            prevPos = rightSlide.getCurrentPosition();

            rightLimitSwitch.update();
            if(rightLimitSwitch.state == LimitSwitch.State.PRESSED){
                rightEncoder = rightLimitSwitch.resetValue;
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
