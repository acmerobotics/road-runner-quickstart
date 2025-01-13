package org.firstinspires.ftc.teamcode.Teleop.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ArmActions {
    public DcMotor leftSlide;
    public DcMotor rightSlide;

    public DcMotor leftHang;
    public DcMotor rightHang;
    public CRServo intake;
    public CRServo subSlide;
    public Servo intakePivot;
    public Servo bucketPivot;
    public Servo clawPivot;
    public Servo claw;


    public ArmActions(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        subSlide = hardwareMap.get(CRServo.class, "sub_extender");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakePivot = hardwareMap.get(Servo.class, "flip");
        bucketPivot = hardwareMap.get(Servo.class, "bucket_pivot");
        clawPivot = hardwareMap.get(Servo.class, "hooks");
        claw = hardwareMap.get(Servo.class, "claw");
        leftHang = hardwareMap.get(DcMotor.class, "leftHang");
        rightHang = hardwareMap.get(DcMotor.class, "rightHang");


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 */


    }


    public Action raiseArm() {
        return new Action() {
            private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(1);
                    leftSlide.setPower(1);
                    initialized = true;
                }

                double pos = leftSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2000) {
                    return true;
                } else {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return false;
                }
            }
        };
    }

    public Action closeClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.4);
                return initialized;
            }
        };
    }

    public Action openClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1);

                return initialized;
            }
        };
    }

    public Action raiseClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawPivot.setPosition(.62);

                return initialized;
            }
        };
    }

    public Action lowerClaw() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    clawPivot.setPosition(.43);
                }

                return initialized;
            }
        };
    }

    public Action lowerArm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(-0.6);
                    leftSlide.setPower(-0.6);
                    initialized = true;
                }

                double pos = leftSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 20) {
                    return true;
                } else {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return false;
                }
            }
        };


    }public Action halfLowerArm() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(-0.4);
                    leftSlide.setPower(-0.4);
                    initialized = true;
                }

                double pos = leftSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1700) {
                    return true;
                } else {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return false;
                }
            }
        };

    }

    public Action smallRaiseArm() {
        return new Action() {
            private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightSlide.setPower(0.5);
                    leftSlide.setPower(0.5);
                    initialized = true;
                }

                double pos = leftSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 150) {
                    return true;
                } else {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return false;
                }
            }
        };
    }

    public Action lowerIntake() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivot.setPosition(0.47);

                return initialized;
            }
        };
    }

    public Action raiseIntake() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivot.setPosition(0);

                return initialized;
            }
        };
    }
    public Action runIntake() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0.5);

                return initialized;
            }
        };
    }
    public Action stopIntake() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);

                return initialized;
            }
        };
    }
    public Action reverseIntake() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-0.5);

                return initialized;
            }
        };
    }
    public Action runSlide() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subSlide.setPower(1);

                return initialized;
            }
        };
    }

    public Action stopSlide() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subSlide.setPower(0);

                return initialized;
            }
        };
    }
    public Action reverseSlide() {
        return new Action() {
            private boolean initialized;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subSlide.setPower(-1);

                return initialized;
            }
        };
    }
}