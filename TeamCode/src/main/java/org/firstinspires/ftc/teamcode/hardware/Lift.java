package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift extends Mechanism{
    /**
     *  lift class that utilizes RoadRunner motion profiling to control movement of lift,
     *  which accesses two motors that are not directly linked, but work together to lift up the
     *  same object.
     */

    //motors
    public DcMotor liftLeft;
    public DcMotor liftRight;

    //retract logic for handling coefficient of gravity. Not needed if utilizing kF
    private boolean retract;
    public static double retMult;

    //height increment for moving slides up and down at steady rate
    public static double HEIGHT_INCREMENT = 1.5;

    // saved lift positions
    public static double maxPos = 15; //highest position slides can reach
    public static double midPos = 9; //mid goal position
    public static double minPos = 0; //lowest position slides can reach (default to 0)

    //PIDCoefficients for RoadRunner motion profiling
    public static PIDCoefficients coeffs = new PIDCoefficients(0.1, 0, 0);
    public static double kF = 0.12; //min power to go against g

    //two controllers because we have two motors (although in theory it is possible to utilize only one
    PIDFController controller;
    PIDFController controller2;

    // lift constants
    public static double SPOOL_DIAMETER_IN = 1.81102;
    public static double MOTOR_RATIO = 5.2;
    public static double TICKS_PER_REV = 145.1;
    public static double GEAR_RATIO = 1.0;

    public static double[] positionHistory = new double[]{10};

    private double targetPosition = 0;

    private boolean inAir = false;
    public void init(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

        // if you need to make sure to reverse as necessary

//        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //makes sure the motors don't move at zero power
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset encoders upon startup
        reset();

        //for using RR PIDs
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * PID Controller values -> see coeffs for more values. Aids power output of motors.
         * kF is frictional coefficient.
         * The lambda expression (x,v) -> {} is essentially saying, "based on position x, and
         * velocity v, set the frictional coefficient to the return value of this function"
         * In this scenario, kF is independent of displacement and velocity, leading to constant kF
         * https://acme-robotics.gitbook.io/road-runner/tour/feedforward-control
         */

        controller = new PIDFController(coeffs, 0, 0, 0, (x,v) -> kF);
        controller2 = new PIDFController(coeffs, 0, 0, 0, (x,v) -> kF);

        /*
         * Part of gravity logic. Not needed if utilizing kF
         */

        retract = false;
        retMult = 0.2;
    }



    /**
     *
     * @param one first double in the comparison
     * @param two second double in the comparison
     * @return true if the two values are within 0.5 range of each other
     */
    private boolean estimatedEqual(double one, double two){
        return Math.abs(one - two) < 0.5;
    }

    /**
     *
     * @return estimation of whether or not the lift is still -> deprecated and potentially unstable
     */
    @Deprecated
    public boolean still(){
        boolean still = true;

        for(int i = 0; i < positionHistory.length-1; i++){
            still = still && estimatedEqual(positionHistory[i],positionHistory[i+1]);
        }

        return still;
    }


    private void updatePID(double target) {
        if(target >= maxPos){target = maxPos;}
        if(target <= minPos){target = minPos;}
        controller.setTargetPosition(target);
        controller2.setTargetPosition(target);
        //find the error
        double leftPow = controller.update(encoderTicksToInches(liftLeft.getCurrentPosition()));
        double rightPow = controller2.update(encoderTicksToInches(liftRight.getCurrentPosition()));
        //compensate error
        if(!retract) {
            liftLeft.setPower(leftPow);
            liftRight.setPower(rightPow);
        }else {
            liftLeft.setPower(leftPow * retMult);
            liftRight.setPower(rightPow * retMult);
        }
    }

    /**
     * updates the lift position utilizing RoadRunner motion profiling. Needs to be called in a loop
     * wrapper for private updatePID method
     */
    public void update() {
        updatePID(targetPosition);
        positionHistory[0] = getCurrentPosition();

        if (positionHistory.length - 1 - 1 >= 0)
            System.arraycopy(positionHistory, 2, positionHistory, 1, positionHistory.length - 1 - 1);
    }

    /**
     *
     * @return returns targetPosition saved -> editable with setTargetPosition
     */
    public double getTargetPosition(){ return targetPosition;}

    /**
     *
     * @return returns encoder values of the current position
     */
    public double getCurrentPosition(){
        return (
                encoderTicksToInches(liftLeft.getCurrentPosition())
                        + encoderTicksToInches(liftRight.getCurrentPosition())
        ) / 2.0;
    }


    /**
     *
     * @param target sets the target position -> retrievable with getTargetPosition
     */
    public void setTargetPosition(double target){targetPosition = target;}

    /**
     * raises slides to midPosition desired (in context of middle goal)
     */
    public void raiseMid(){
        setTargetPosition(midPos);
        inAir = true;
    }

    /**
     * raises slides to highPosition desired (in context of high goal)
     */
    public void raiseHigh(){
        setTargetPosition(maxPos);
        inAir = true;
    }

    /**
     * lowers slides
     */
    public void lower(){
        setTargetPosition(minPos);
        inAir = false;

    }

    /**
     * toggles slides (DEPRECATED due to use of scoringV2)
     */

    @Deprecated
    public void toggle(){
        if(inAir){
            lower();
        }
        else{
            raiseHigh();
        }
    }

    /**
     *
     * @return returns true if the slides are not at minPosition
     */
    @Deprecated
    public boolean inAir(){return inAir;}

    /**
     * extends the slides at a constant rate
     */
    @Deprecated
    public void extend(){
        retracting(false);
        setTargetPosition(Math.min(this.getTargetPosition() + HEIGHT_INCREMENT, maxPos));
    }

    /**
     * lowers the slides at a constant rate
     */
    @Deprecated
    public void retract(){
        retracting(true);
        setTargetPosition(Math.max(this.getTargetPosition() - HEIGHT_INCREMENT, minPos));
    }

    /**
     *
     * @param ticks number of ticks of desired encoder
     * @return calculated angular displacement in inches of spool
     */
    private static double encoderTicksToInches(double ticks) {
        return SPOOL_DIAMETER_IN  * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    /**
     *
     * sets status of slide retraction. Important if ret multiplier is utilized instead of kF constant
     */
    public void retracting(boolean status) {
        retract = status;
        if(retract) kF = 0;
        else kF = 0.12;
    }

    public void reset(){
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}