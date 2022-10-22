package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionProfileDebug;
import org.firstinspires.ftc.teamcode.Utils.ProfiledServo;


@Config
public class ScoringMechanism extends Subsystem {


    // LONG = side closets to the slide of the robot itself, arm reaches out the longest
    // SHORT = vice versa


    private static final double CUTOFF_POINT = 4; // min height of slides for arm to move over the robot.
    public static double WRIST_COLLECT_SHORT = 0.0;
    public static double WRIST_COLLECT_LONG = 0;
    public static double WRIST_STOW = 0.9;
    public static double WRIST_CARRY_SHORT = 0.2;
    public static double WRIST_DEPOSIT_LONG = WRIST_STOW;

    public static double ARM_IN_COLLECT = 0.0;
    public static double ARM_CARRY = 0.1;
    public static double ARM_DEPOSIT_LONG_HIGH = 0.35;
    public static double ARM_DEPOSIT_LONG_MID = 0.6;
    public static double ARM_DEPOSIT_LONG_LOW = 0.6;

    public static double ARM_DEPOSIT_SHORT_HIGH = 0.6;
    public static double ARM_DEPOSIT_SHORT_MID = 0.6;
    public static double ARM_DEPOSIT_SHORT_LOW = 0.6;

    public static double INTAKE_SPEED_HOLD = 0.2;
    public static double INTAKE_SPEED = 0.5;
    public static double OUT_TAKE = -1;

    public static double SLIDES_IN = 0;
    public static double SLIDES_CLEAR = 4;
    public static double SLIDES_HIGH = 17;
    public static double SLIDES_MID = 10;
    public static double SLIDES_LOW = 5;

    protected double currentWristPos = WRIST_STOW;
    protected double currentArmPos = ARM_CARRY;
    protected double currentMotorTarget = 0;
    protected double intakePower = INTAKE_SPEED_HOLD;

    protected boolean should_traverse = false;  // should traverse to the next state in the state machine

    public static boolean LONG_OUT_DEFAULT = true; // if true, in-taking on short side, out take on long is default

    protected States state = States.STOW; // state variable

    protected DcMotorEx slideLeft;
    protected DcMotorEx slideRight;


    protected PIDCoefficients coefficients = new PIDCoefficients(0.45,0,0);
    ElapsedTime slide_profile_timer = new ElapsedTime();

    public MotionConstraint slide_constraints = new MotionConstraint(35,25,40);

    protected AsymmetricMotionProfile profile_slides = new AsymmetricMotionProfile(0,0,slide_constraints);
    protected FeedbackController slideControllerLeft = new BasicPID(coefficients);
    protected FeedbackController slideControllerRight = new BasicPID(coefficients);

    protected Servo wrist;
    protected ProfiledServo arm;
    protected CRServo intake;

    protected States desiredEnd = States.HIGH;
    protected States desiredEndTransition = States.GO_TO_HIGH;

    ElapsedTime TraverseTimer = new ElapsedTime();  // timer used to help assist some servo position specific maneuvers such as putting down the arm.

    double GO_TO_INTAKE_TIME = 0.5; // time between fully out-taking and moving arm before slides go back down to prevent bad things
    double OUTTAKE_DURATION = 0.25;  // time the out take occurs for before putting slides back in.
    private double previousMotorTarget = 10000000;

    private void commonInit(HardwareMap hwMap) {
        slideLeft = hwMap.get(DcMotorEx.class, "left_lift");
        slideRight = hwMap.get(DcMotorEx.class, "right_lift");
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double velocityForward = 0.8; //percent/s
        double accelForward = 1; // percent/s^2

        double velocityBackward = 0.5;
        double accelBackward = 0.7;


        arm = new ProfiledServo(hwMap, "arm_left","arm_right",velocityForward,accelForward * 1.5,accelForward / 2,velocityBackward,accelBackward * 1.5,accelBackward / 2,ARM_IN_COLLECT);

        wrist = hwMap.get(Servo.class, "wrist");

        intake = hwMap.get(CRServo.class, "intake");
        setServoPositions(true);
        state = States.CARRY;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);

        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void periodic() {

        transitionLogic();
        setPositions();
        updateProfiledServos();

    }


    protected void updateProfiledServos() {
        arm.periodic();
    }

    /**
     * wow
     */
    public void transitionLogic() {
        switch (state) {
            case STOW:
                commandActuatorSetpoints(WRIST_STOW,ARM_IN_COLLECT,SLIDES_IN,INTAKE_SPEED_HOLD);
                if (should_traverse) {
                    state = States.CARRY;
                    should_traverse = false;
                }
                break;
            case INTAKE_ON:
                commandActuatorSetpoints(WRIST_COLLECT_SHORT,ARM_IN_COLLECT,SLIDES_IN,INTAKE_SPEED);
                if (should_traverse) {
                    should_traverse = false;
                    state = States.CARRY;
                }
                break;
            case CARRY:
                commandActuatorSetpoints(WRIST_CARRY_SHORT,ARM_CARRY,SLIDES_IN,INTAKE_SPEED_HOLD);
                if (should_traverse) {
                    state = desiredEndTransition;
                }
                break;
            case GO_TO_HIGH:
            case GO_TO_MID:
            case GO_TO_LOW:
                commandActuatorSetpoints(WRIST_DEPOSIT_LONG,ARM_IN_COLLECT,getDesiredHeight(desiredEnd),INTAKE_SPEED_HOLD);
                if (getSlideHeightIN() > CUTOFF_POINT) {
                    state = desiredEnd;
                    should_traverse = false;
                }
                break;
            case HIGH:
            case MID:
            case LOW:
                commandActuatorSetpoints(WRIST_DEPOSIT_LONG,getDesiredArmPos(desiredEnd),getDesiredHeight(desiredEnd),INTAKE_SPEED_HOLD);
                if (should_traverse) {
                    state = States.DEPOSIT;
                    should_traverse = false;
                    TraverseTimer.reset();
                }
                break;
            case DEPOSIT:
                commandActuatorSetpoints(WRIST_DEPOSIT_LONG,getDesiredArmPos(desiredEnd),getDesiredHeight(desiredEnd),OUT_TAKE);
                if (TraverseTimer.seconds() > OUTTAKE_DURATION) {
                    state = States.GO_TO_INTAKE;
                    TraverseTimer.reset();
                }
            case GO_TO_INTAKE:
                commandActuatorSetpoints(WRIST_DEPOSIT_LONG,getDesiredArmPos(desiredEnd),getDesiredHeight(desiredEnd),OUT_TAKE);

                if (TraverseTimer.seconds() > GO_TO_INTAKE_TIME) {
                    state = States.CARRY;
                    should_traverse = false;
                }
                break;
        }
        System.out.println("Current state: " + state);
    }

    /**
     *
     * @param s - Low, Medium, High desired state
     * @return gives the arm angle in percent (0-1)
     */
    public double getDesiredArmPos(States s) {
        switch (s) {
            case HIGH:
                if (LONG_OUT_DEFAULT) {
                    return ARM_DEPOSIT_LONG_HIGH;
                }
                return ARM_DEPOSIT_LONG_LOW;
            case MID:
                if (LONG_OUT_DEFAULT) {
                    return ARM_DEPOSIT_LONG_MID;
                }
                return ARM_DEPOSIT_SHORT_MID;
            default:
                if (LONG_OUT_DEFAULT) {
                    return ARM_DEPOSIT_LONG_LOW;
                }
                return ARM_DEPOSIT_SHORT_LOW;

        }
    }


    public void setServoPositions() {
        setArmPosition(currentArmPos);
        intake.setPower(intakePower);
        wrist.setPosition(currentWristPos);
        intake.setPower(intakePower);
    }


    public void setServoPositions(boolean isInit) {
        setArmPosition(currentArmPos);
        intake.setPower(intakePower);
        wrist.setPosition(currentWristPos);
        if (!isInit){
            intake.setPower(intakePower);
        }
    }

    public void setPositions() {
        setServoPositions();
        regenerate_slide_profile();
        double slide_profile_position = profile_slides.calculate(slide_profile_timer.seconds()).getX();
        System.out.println("slide position from motion profile: " + slide_profile_position);
        slideControl(slide_profile_position);
    }

    @Override
    public void shutdown() {
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    // TODO I think these calculations are wrong
    public static double encoderTicksToInches(double ticks) {
        double SPOOL_SIZE_IN = 1.270 / 2.0;
        double GEAR_RATIO = 1;
        return (SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks) / (28 * 13.7);
    }

    protected void slideControl(double ticks) {
        double left = encoderTicksToInches(slideLeft.getCurrentPosition());
        double right = encoderTicksToInches(slideRight.getCurrentPosition());
        System.out.println("left motor pos: " + left + " right motor pos: " + right);
        previousMotorTarget = currentMotorTarget;

        double leftCommand = slideControllerLeft.calculate(ticks,left);
        double rightCommand = slideControllerRight.calculate(ticks,right);
        setSlidePower(leftCommand,rightCommand);

    }

    protected void setArmPosition(double position) {
        arm.setPosition(position);
//        left_arm.setPosition(1 - position);
//        right_arm.setPosition(position);
    }

    public void setSlidePower(double leftPower,double rightPower) {
        slideLeft.setPower(leftPower);
        slideRight.setPower(rightPower);
    }

    public enum States {
        STOW,
        INTAKE_READY,
        INTAKE_ON,
        CARRY,
        GO_TO_HIGH,
        GO_TO_MID,
        GO_TO_LOW,
        HIGH,
        MID,
        LOW,
        DEPOSIT,
        GO_TO_INTAKE
    }


    protected void commandActuatorSetpoints(double wrist, double arm, double slides, double intake) {
        this.currentWristPos = wrist;
        this.currentArmPos = arm;
        this.currentMotorTarget = slides;
        this.intakePower = intake;
    }


    /**
     * external method that allows changing the desired height
     *
     * will only let height change occur if slides are in a "ground" state
     *
     * @param s desired state
     */
    public void setTarget(States s) {


        boolean allow_change = state.equals(States.STOW) ||
                                state.equals(States.INTAKE_READY) ||
                                state.equals(States.INTAKE_ON) ||
                                state.equals(States.CARRY);

        if (!allow_change) {
            return;
        }

        switch (s) {
            case HIGH:
                desiredEnd = States.HIGH;
                desiredEndTransition = States.GO_TO_HIGH;
                break;
            case MID:
                desiredEnd = States.MID;
                desiredEndTransition = States.GO_TO_MID;
                break;
            default:
                desiredEnd = States.LOW;
                desiredEndTransition = States.GO_TO_LOW;
                break;
        }
        should_traverse = true;

    }


    public States getState() {
        return state;
    }

    /**
     *
     * @return average of two slide positions in inches
     */
    public double getSlideHeightIN() {
        double s1 = encoderTicksToInches(slideLeft.getCurrentPosition());
        double s2 = encoderTicksToInches(slideRight.getCurrentPosition());
        return (s1 + s2) / 2;
    }

    /**
     *
     * @param s - state variable
     * @return slide position in inches
     */
    public double getDesiredHeight(States s) {
        switch (s) {
            case HIGH:
                return SLIDES_HIGH;
            case MID:
                return SLIDES_MID;
            default:
                return SLIDES_LOW;
        }
    }

    /**
     * if in placing position, this will allow the traversal to the next state which is out taking
     */
    public void activateEjection() {
        if (state.equals(States.HIGH) || state.equals(States.MID) || state.equals(States.LOW)) {
            should_traverse = true;
        }
    }

    public void ACTIVATE_INTAKE() {
        if (state.equals(States.CARRY)) {
            state = States.INTAKE_ON;
            should_traverse = false;
        }
    }

    public void STOP_INTAKE() {
        if (state.equals(States.INTAKE_ON)) {
            should_traverse = true;
        }
    }

    public void GO_TO_SCORING() {
        if (state.equals(States.CARRY)) {
            should_traverse = true;
        }
    }


    public boolean isArmBusy() {
        return arm.isBusy();
    }

    protected void regenerate_slide_profile() {
        if (currentMotorTarget != previousMotorTarget) {
            profile_slides = new AsymmetricMotionProfile(getSlideHeightIN(),currentMotorTarget,slide_constraints);
            slide_profile_timer.reset();
        }
        previousMotorTarget = currentMotorTarget;
    }

}