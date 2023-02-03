package org.firstinspires.ftc.teamcode.oldCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class RobotHardware {

    public BNO055IMU imu;

//    public DcMotor motorCarousel;


    public DriveTrain driveTrain;

    public static Lift lift;


    public static final double WHEEL_DIAMETER = 6.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 385.5; //changing from 537.6


    private HardwareMap hardwareMap;

    public RobotHardware(HardwareMap aHardwareMap, boolean initIMU) {
        hardwareMap = aHardwareMap;

        driveTrain = new DriveTrain(hardwareMap);
        lift = new Lift(hardwareMap);

        if (initIMU) {
            initializeIMU();
        }


    }

    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitImuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        //parameters.loggingTag          = "IMU";
        //parameters.mode                = BNO055IMU.SensorMode.NDOF;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Config
    public static class Lift {
        public DcMotor motorLiftL;

        public DcMotor motorTurret;
        public DcMotor motorLiftR;

        public Servo servoClaw;

        public Servo servoExtension;




        public States currentState = States.INTAKE;

        ElapsedTime elapsedTime;

        public static int LIFT_HIGH_POS = 3100;
        public static int LIFT_MID_POS = 2150;
        public static int LIFT_LOW_POS = 1250;
        public static int LIFT_HOVER_POS  = 100;
        public static int LIFT_INTAKE_POS = 0;

        public static int TURRET_LEFT_POS = 1545;
        public static int TURRET_RIGHT_POS = -1560;
        public static int TURRET_LEFT_135_POS = 2300;
        public static int TURRET_RIGHT_135_POS = -2350;
        public static int TURRET_LEFT_45_POS = 770;
        public static int TURRET_RIGHT_45_POS = -780;
        public static int TURRET_STACK_POS = -2700;
        public static int TURRET_180_POS = 3080;
        public static int TURRET_0_POS = 0;

        public static double CLAW_CLOSE_POS = 0.52;
        public static double CLAW_OPEN_POS = 0.72;
        public static double CLAW_INIT_POS = 0.78;

        public static double EXTENSION_SCORE_L_POS = 0.39;
        public static double EXTENSION_SCORE_R_POS = 0.39;
        public static double EXTENSION_180_SCORE_POS = 0.52;
        public static double EXTENSION_INTAKE_POS = 0.26;
        public static double EXTENSION_INIT_POS = 0.26;

        public static double EXTENSION_INTAKE_OUT_POS = 0.6;
        public static double EXTENSION_SCORE_45_POS = 0.52;
        public static double EXTENSION_AUTO_POS = 0.6;
        public static double EXTENSION_90_INTAKE_POS = 0.52;
        public static double EXTENSION_90_AUTO_POS = 0.34;
        public static double EXTENSION_90_AUTO_LOW_POS = 0.36;


        public Lift (HardwareMap hardwareMap) {
            motorLiftL = hardwareMap.get(DcMotor.class, "motorLiftL");

            motorLiftL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorLiftR = hardwareMap.get(DcMotor.class, "motorLiftR");

            motorLiftR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");

            motorTurret.setDirection(DcMotorSimple.Direction.FORWARD);
            motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            servoClaw = hardwareMap.get(Servo.class, "servoClaw");
            servoClaw.setPosition(CLAW_INIT_POS);

            servoExtension = hardwareMap.get(Servo.class, "servoExtension");
            servoExtension.setPosition(EXTENSION_INIT_POS);



//            ElapsedTime elapsedTime = new ElapsedTime();

        }

        public void HOVER() {
            motorLiftL.setTargetPosition(LIFT_HOVER_POS);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setPower(1);
            motorLiftR.setTargetPosition(LIFT_HOVER_POS);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setPower(1);
            servoExtension.setPosition(EXTENSION_INTAKE_POS);
            currentState = States.HOVER;
        }
        public void HighJunction() {
                motorLiftL.setTargetPosition(LIFT_HIGH_POS);
                motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftL.setPower(1);
                motorLiftR.setTargetPosition(LIFT_HIGH_POS);
                motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftR.setPower(1);
                currentState = States.HighJunction;
//                elapsedTime.reset();

        }


        public void MiddleJunction() {
            motorLiftL.setTargetPosition(LIFT_MID_POS);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setPower(1);
            motorLiftR.setTargetPosition(LIFT_MID_POS);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setPower(1);
            currentState = States.MiddleJunction;
        }

        public void LowJunction() {
            motorLiftL.setTargetPosition(LIFT_LOW_POS);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setPower(1);
            motorLiftR.setTargetPosition(LIFT_LOW_POS);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setPower(1);
            currentState = States.LowJunction;
        }

        public void Intake() {
            motorLiftL.setTargetPosition(10);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setPower(1);
            motorLiftR.setTargetPosition(10);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setPower(1);
            motorTurret.setTargetPosition(TURRET_0_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.7);
            servoExtension.setPosition(EXTENSION_INTAKE_POS);
            currentState = States.INTAKE;

        }

        public void TurretRightIntake(){
            motorLiftL.setTargetPosition(0);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setPower(0.8);
            motorLiftR.setTargetPosition(0);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setPower(0.8);
            motorTurret.setTargetPosition(TURRET_RIGHT_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.5);
            servoExtension.setPosition(EXTENSION_90_INTAKE_POS);
            currentState = States.TurretRightINTAKE;
        }

        public void TurretLeftIntake(){
            motorLiftL.setTargetPosition(0);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setPower(0.8);
            motorLiftR.setTargetPosition(0);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setPower(0.8);
            motorTurret.setTargetPosition(TURRET_LEFT_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.5);
            servoExtension.setPosition(EXTENSION_90_INTAKE_POS);
            currentState = States.TurretLeftINTAKE;
        }

        public void TurretLeft() {
            motorTurret.setTargetPosition(TURRET_LEFT_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.7);
            servoExtension.setPosition(EXTENSION_SCORE_L_POS);
            currentState = States.TurretLeft;

        }

        public void TurretRight() {
            motorTurret.setTargetPosition(TURRET_RIGHT_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.7);
            servoExtension.setPosition(EXTENSION_SCORE_R_POS);
            currentState = States.TurretRight;
        }

        public void Turret180Cycle() {
            motorLiftL.setTargetPosition(LIFT_HIGH_POS);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setPower(1);
            motorLiftR.setTargetPosition(LIFT_HIGH_POS);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setPower(1);
            motorTurret.setTargetPosition(TURRET_180_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.7);
            servoExtension.setPosition(EXTENSION_180_SCORE_POS);
            currentState = States.Turret180Cycle;
        }

        public void Turret180() {
            motorTurret.setTargetPosition(TURRET_180_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.7);
            servoExtension.setPosition(EXTENSION_INTAKE_POS);
            currentState = States.Turret180;
        }

        public void Turret0() {
            motorTurret.setTargetPosition(TURRET_0_POS);
            motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTurret.setPower(0.7);
            servoExtension.setPosition(EXTENSION_INTAKE_POS);
            currentState = States.Turret0;
        }

        public void run() {
            if (currentState == States.HOVER) {

            }
        }


        enum States {
            INTAKE,
            HOVER,
            HighJunction,
            MiddleJunction,
            LowJunction,

            TurretLeft,
            TurretRight,
            Turret180,
            Turret0,

            TurretRightINTAKE,
            TurretLeftINTAKE,
            Turret180Cycle,
            MANUAL_MODE

        }

        enum Level {
            TOP,
            MIDDLE,
            BOTTOM,
            FLOOR
        }
    }

    public class DriveTrain {
        public DcMotor motorFL;
        public DcMotor motorFR;
        public DcMotor motorBL;
        public DcMotor motorBR;
        public DcMotor[] motors;



        public DriveTrain(HardwareMap hardwareMap) {
            motorFL = hardwareMap.get(DcMotor.class, "motorFL");
            motorFR = hardwareMap.get(DcMotor.class, "motorFR");
            motorBL = hardwareMap.get(DcMotor.class, "motorBL");
            motorBR = hardwareMap.get(DcMotor.class, "motorBR");
            motors = new DcMotor[]{motorFL, motorFR, motorBL, motorBR};


            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        public void startMove(double drive, double strafe, double turn, double scale) {
            double powerFL = (drive + strafe + turn) * scale;
            double powerFR = (drive - strafe - turn) * scale;
            double powerBL = (drive - strafe + turn) * scale;
            double powerBR = (drive + strafe - turn) * scale;

            double maxPower = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)), Math.max(Math.abs(powerBL), Math.abs(powerBR)));
            double max = (maxPower < 1) ? 1 : maxPower;

            motorFL.setPower(Range.clip(powerFL / max, -1, 1));
            motorFR.setPower(Range.clip(powerFR / max, -1, 1));
            motorBL.setPower(Range.clip(powerBL / max, -1, 1));
            motorBR.setPower(Range.clip(powerBR / max, -1, 1));
        }

        public void stopMove() {
            for (DcMotor motor: motors) {
                motor.setPower(0);
            }
        }

        public void resetDriveEncoders() {
            for (DcMotor motor: motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void telemetryUpdate(Telemetry telemetry) {
            telemetry.addData("BL pos", motorBL.getCurrentPosition());
            telemetry.addData("BR pos", motorBR.getCurrentPosition());
            telemetry.addData("FR pos", motorFR.getCurrentPosition());
            telemetry.addData("FL pos", driveTrain.motorFL.getCurrentPosition());
        }
    }

    public static class Sensors {
        public DistanceSensor colorSensor;

        public Sensors(HardwareMap hardwareMap) {
            colorSensor = hardwareMap.get(DistanceSensor.class, "sensorColor");
        }

        public void telemetryUpdate(Telemetry telemetry) {
            telemetry.addData("Color Distance", colorSensor.getDistance(DistanceUnit.INCH));

        }
    }

}


