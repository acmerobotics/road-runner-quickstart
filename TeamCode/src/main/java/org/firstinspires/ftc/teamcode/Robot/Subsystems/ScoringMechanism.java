package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Utils.ProfiledServo;


@Config
public class ScoringMechanism extends Subsystem {


    // LONG = side closets to the slide of the robot itself, arm reaches out the longest
    // SHORT = vice versa


    private static final double CUTOFF_POINT = 4; // min height of slides for arm to move over the robot.
    public static double WRIST_COLLECT_SHORT = 0.15;
    public static double WRIST_COLLECT_LONG = 0;
    public static double WRIST_STOW = 1;
    public static double WRIST_CARRY_SHORT = 0.2;
    public static double WRIST_DEPOSIT_LONG = 1;

    public static double ARM_IN_COLLECT = 0.0;
    public static double ARM_CARRY = 0.1;
    public static double ARM_DEPOSIT_LONG_HIGH = 0.38;
    public static double ARM_DEPOSIT_LONG_MID = 0.38;
    public static double ARM_DEPOSIT_LONG_LOW = 0.6;

    public static double ARM_DEPOSIT_SHORT_HIGH = 0.6;
    public static double ARM_DEPOSIT_SHORT_MID = 0.6;
    public static double ARM_DEPOSIT_SHORT_LOW = 0.6;

    public static double INTAKE_SPEED_HOLD = 0.1;
    public static double INTAKE_SPEED = 0.85;
    public static double OUT_TAKE = -0.85;

    public static double SLIDES_IN = 0;
    public static double SLIDES_CLEAR = 4;
    public static double SLIDES_HIGH = 16.5;
    public static double SLIDES_MID = 8;
    public static double SLIDES_LOW = 5;
    public static double SLIDES_SAFE_FOR_STACK = 5;

    protected double currentWristPos = WRIST_STOW;
    protected double currentArmPos = ARM_CARRY;
    protected double currentMotorTarget = 0;
    protected double intakePower = INTAKE_SPEED_HOLD;

    protected boolean should_traverse = false;  // should traverse to the next state in the state machine

    public static boolean LONG_OUT_DEFAULT = true; // if true, in-taking on short side, out take on long is default

    protected States state = States.STOW; // state variable

    protected DcMotorEx slideLeft;
    protected DcMotorEx slideRight;

    protected States currentStackProgress = States.AUTO_INTAKE_5; // the 5 high stack is the first.

    protected LowPassFilter intake_power_filter = new LowPassFilter(0.5);


    protected PIDCoefficients coefficients = new PIDCoefficients(0.45,0,0);
    protected PIDCoefficients coefficients_between = new PIDCoefficients(0.6,0,0);
    ElapsedTime slide_profile_timer = new ElapsedTime();

    public MotionConstraint slide_constraints = new MotionConstraint(45,30,50);

    protected AsymmetricMotionProfile profile_slides = new AsymmetricMotionProfile(0,0,slide_constraints);
    protected FeedbackController slideControllerLeft = new BasicPID(coefficients);
    protected FeedbackController slideControllerRight = new BasicPID(coefficients);
    protected FeedbackController betweenSlideController = new BasicPID(coefficients_between);

    protected Servo wrist;
    protected ProfiledServo arm;
    protected CRServo intake;

    protected States desiredEnd = States.HIGH;
    protected States desiredEndTransition = States.GO_TO_HIGH;

    protected States desiredIntakingType = States.CARRY;

    ElapsedTime TraverseTimer = new ElapsedTime();  // timer used to help assist some servo position specific maneuvers such as putting down the arm.

    double GO_TO_INTAKE_TIME = 0.6; // time between fully out-taking and moving arm before slides go back down to prevent bad things
    double OUTTAKE_DURATION = 0.6;  // time the out take occurs for before putting slides back in.
    private double previousMotorTarget = 10000000;
    private double AUTO_INTAKE_DURATION = 0.5;

    /**
     * The initialization for both autonomous and teleop
     * @param hwMap the hardware map
     */
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
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setServoPositions(false);
        state = States.CARRY;
    }

    /**
     * The initialization specifically for autonomous
     * @param hwMap the hardware map
     */
    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);

        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * The initialization specifically for teleop
     * @param hwMap the hardware map
     */
    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * The update loop for both autonomous and teleop
     */
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
     * The main logic for the state machine
     * This is where the state transitions are defined.
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
                    state = desiredIntakingType;
                    should_traverse = false;
                }
                break;
            case AUTO_INTAKE_SAFE: // at safe height to approach stack and then begin intaking
                commandActuatorSetpoints(WRIST_COLLECT_SHORT, ARM_IN_COLLECT, SLIDES_SAFE_FOR_STACK, INTAKE_SPEED_HOLD);
                if (should_traverse) {
                    should_traverse = false;
                    state = currentStackProgress;
                    TraverseTimer.reset();
                }
                break;
            case AUTO_INTAKE_5:
            case AUTO_INTAKE_4:
            case AUTO_INTAKE_3:
            case AUTO_INTAKE_2:
            case AUTO_INTAKE_1:
                commandActuatorSetpoints(WRIST_COLLECT_SHORT, ARM_IN_COLLECT, getSlideHeightForAutoIntaking(),INTAKE_SPEED);
                if (TraverseTimer.seconds() > 2) {
                    currentStackProgress = getNextAutoIntake();
                    state = States.AUTO_STOP_IN_TAKING;
                }
                break;
            case AUTO_STOP_IN_TAKING:
                commandActuatorSetpoints(WRIST_COLLECT_SHORT, ARM_IN_COLLECT, SLIDES_SAFE_FOR_STACK,INTAKE_SPEED_HOLD);
                if (getSlideHeightIN() > getSlideHeightForAutoIntaking()) {
                    state = States.READY_TO_SCORE_AUTO;
                }
                break;
            case READY_TO_SCORE_AUTO:
                commandActuatorSetpoints(WRIST_CARRY_SHORT,ARM_CARRY,SLIDES_SAFE_FOR_STACK,INTAKE_SPEED_HOLD);
                if (should_traverse) {
                    state = desiredEndTransition;
                }
                break;
        }
    }

    /**
     * A getter for finding the desired angle of the entire arm.
     * Checks LONG_OUT_DEFAULT to determine the angle
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
                return ARM_DEPOSIT_SHORT_HIGH;
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

    /**
     * Sets the desired angle of the arm, sets the desired speed of the intake, and sets the position of the wrist
     * Based on object variables
     */
    public void setServoPositions() {
        setArmPosition(currentArmPos);
        intake.setPower(intake_power_filter.estimate(intakePower));
        System.out.println("set intake to power: " + intakePower + " at Current state: " + state);
        System.out.println();

        wrist.setPosition(currentWristPos);
    }

    /**
     * Sets the desired angle of the arm, sets the desired speed of the intake, and sets the position of the wrist
     * Based on object variables
     */
    public void setServoPositions(boolean isInit) {
        setArmPosition(currentArmPos);
        wrist.setPosition(currentWristPos);
        if (!isInit){
            intake.setPower(intakePower);
        }
    }

    /**
     * Sets all of the positions of the items
     */
    public void setPositions() {
        setServoPositions();
        regenerate_slide_profile();
        double slide_profile_position = profile_slides.calculate(slide_profile_timer.seconds()).getX();
        System.out.println("slide position from motion profile: " + slide_profile_position);
        slideControl(slide_profile_position);
    }

    /**
     * Shutdowns the scoring mechanism, stopping the slides
     */
    @Override
    public void shutdown() {
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    /**
     * Sets the wrist position
     * @param position the position to set the wrist to
     */
    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    // TODO I think these calculations are wrong

    /**
     * Gets the amount of inches for given ticks
     * @param ticks the amount of ticks
     * @return the amount of inches
     */
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
        double betweenSlideOutput = betweenSlideController.calculate(left,right);
        double leftCommand = slideControllerLeft.calculate(ticks,left) - betweenSlideOutput;
        double rightCommand = slideControllerRight.calculate(ticks,right) + betweenSlideOutput;
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
        INTAKE_ON,
        CARRY,
        GO_TO_HIGH,
        GO_TO_MID,
        GO_TO_LOW,
        HIGH,
        MID,
        LOW,
        DEPOSIT,
        GO_TO_INTAKE,
        AUTO_INTAKE_SAFE, // above the stack safe distance
        AUTO_INTAKE_5,
        AUTO_INTAKE_4,
        AUTO_INTAKE_3,
        AUTO_INTAKE_2,
        AUTO_INTAKE_1,
        AUTO_STOP_IN_TAKING, // after in-taking go to this state to put the slides up
        READY_TO_SCORE_AUTO // after auto in-aking, go to this state to signify it is safe to move towards the junction
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
    public void activateEjectionTeleop() {
        if (state.equals(States.HIGH) || state.equals(States.MID) || state.equals(States.LOW)) {
            should_traverse = true;
            desiredIntakingType = States.CARRY;
        }
    }

    public void activateEjectionAuto() {
        if (state.equals(States.HIGH) || state.equals(States.MID) || state.equals(States.LOW)) {
            should_traverse = true;
            desiredIntakingType = States.AUTO_INTAKE_SAFE;
        }
    }

    public void ACTIVATE_INTAKE() {
        if (state.equals(States.CARRY)) {
            state = States.INTAKE_ON;
            should_traverse = false;
        }
    }

    public void ACTIVATE_INTAKE_AUTO() {
        if (state.equals(States.AUTO_INTAKE_SAFE)) {
            state = currentStackProgress;
            should_traverse = false;
        }
    }

    public void STOP_INTAKE() {
        if (state.equals(States.INTAKE_ON)) {
            should_traverse = true;
        }
    }

    public void GO_TO_SCORING() {
        if (state.equals(States.CARRY) || state.equals(States.READY_TO_SCORE_AUTO)) {
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

    public States getNextAutoIntake() {
        switch (currentStackProgress) {
            case AUTO_INTAKE_5:
                return States.AUTO_INTAKE_4;
            case AUTO_INTAKE_4:
                return States.AUTO_INTAKE_3;
            case AUTO_INTAKE_3:
                return States.AUTO_INTAKE_2;
            default:
                return States.AUTO_INTAKE_1;
        }
    }
    public double getSlideHeightForAutoIntaking() {
        switch (currentStackProgress) {
            case AUTO_INTAKE_5:
                return 3;
            case AUTO_INTAKE_4:
                return 2;
            case AUTO_INTAKE_3:
                return 1;
            default:
                return 0;
        }

    }
}