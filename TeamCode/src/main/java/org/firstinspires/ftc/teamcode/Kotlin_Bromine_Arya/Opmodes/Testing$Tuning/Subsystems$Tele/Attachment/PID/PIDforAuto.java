package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Testing$Tuning.Subsystems$Tele.Attachment.PID;

//TODO(Add ftc lib to gradle, or change the pid being used to our custom version)
/*
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Autonomous(name="PIDforAuto", group="Linear OpMode")
public class PIDforAuto extends LinearOpMode {

    public static double[] p = new double[4];
    public static double[] i = new double[4];
    public static double[] d = new double[4];

    //in inches
    public static int target=0;
    public double ENCtoINC =(9.6/2.54* Math.PI)/537.689839572;

    public DcMotorEx rightFront;
    public DcMotorEx leftFront;
    public DcMotorEx rightBack;
    public DcMotorEx leftBack;

    double currentxpos = 0;
    double currentypos = 0;
    double currentyawpos = 0;

    double Axial;
    double Lateral;
    double Yaw;
    @Override
    public void runOpMode() {


        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();






    }
    private void Movefor (double xpos, double ypos, double yawpos, double speedmult, double accuracy) {



        p[0] = .006;
        i[0] = 0;
        d[0] = .0001;

        PIDController X = new PIDController(p[0], i[0], d[0]);

        X.setPID(p[0], i[0], d[0]);

        p[1] = .006;
        i[1] = 0;
        d[1] = .0001;

        PIDController Y = new PIDController(p[1], i[1], d[1]);


        Y.setPID(p[1], i[1], d[1]);
        PIDController Ya = new PIDController(p[2], i[2], d[2]);

        p[2] = .006;
        i[2] = 0;
        d[2] = .0001;

        Ya.setPID(p[2], i[2], d[2]);

        while ((Axial > 0 + accuracy && Axial < 0 - accuracy) && (Lateral > 0 + accuracy && Lateral < 0 - accuracy) && (Yaw > 0 + accuracy && Yaw < 0 - accuracy)) {

            //howfinddeez
            currentxpos = 0;
            currentypos = 0;
            currentyawpos = 0;

            Axial = X.calculate(currentxpos, xpos);
            Lateral = Y.calculate(currentypos, ypos);
            Yaw = Ya.calculate(currentyawpos, yawpos);


            double proportion = Math.max(Axial + Lateral + Yaw, Axial - Lateral + Yaw);
            proportion = Math.max(proportion, Axial - Lateral - Yaw);
            proportion = Math.max(proportion, Axial + Lateral - Yaw);

            leftFront.setPower(speedmult * (Axial + Lateral + Yaw) / proportion);
            leftBack.setPower(speedmult * (Axial - Lateral + Yaw) / proportion);
            rightFront.setPower(speedmult * (Axial - Lateral - Yaw) / proportion);
            rightBack.setPower(speedmult * (Axial + Lateral - Yaw) / proportion);


            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
}


 */