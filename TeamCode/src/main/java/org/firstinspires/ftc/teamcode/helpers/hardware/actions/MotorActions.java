package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.data.PositionsClass;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;

public class MotorActions {
    public final MotorControl motorControl;
    public PositionsClass.Intake intakePosition = PositionsClass.Intake.Transfer;
    public PositionsClass.OutTake outtakePosition = PositionsClass.OutTake.Transfer;

    public final Extendo extendo;
    public final Lift lift;
    public final IntakeArm intakeArm;
    public final IntakePivot intakePivot;
    public final IntakeClaw intakeClaw;
    public final OuttakeArm outtakeArm;
    public final OuttakePivot outtakePivot;
    public final OutTakeClaw outTakeClaw;
    public final IntakeTurret intakeTurret;
    public final MecanumDrive drive;

    public MotorActions(MotorControl motorControl, MecanumDrive drive) {
        this.drive = drive;
        this.motorControl = motorControl;
        this.extendo = new Extendo();
        this.lift = new Lift();
        this.intakeArm = new IntakeArm();
        this.intakePivot = new IntakePivot();
        this.outtakeArm = new OuttakeArm();
        this.outtakePivot = new OuttakePivot();
        this.intakeClaw = new IntakeClaw();
        this.outTakeClaw = new OutTakeClaw();
        this.intakeTurret = new IntakeTurret();
    }

    public Action intakeExtend(double Position, int index) {
        return new SequentialAction(
                t -> {
                    intakePosition = PositionsClass.Intake.Extended;
                    return false;
                },
                intakeTurret.setPositionByIndex(index),
                intakeArm.Extended(),
                intakePivot.Grab(),
                extendo.setTargetPosition(Position),
                extendo.waitUntilFinished(),
                intakeArm.waitUntilFinished(),
                intakeClaw.Open());
    }


    public Action intakeTransfer() {
        return new SequentialAction(
                t -> {
                    intakePosition = PositionsClass.Intake.Transfer;
                    return false;
                },
                intakeArm.Grab(),
                intakeArm.waitUntilFinished(),
                new SleepAction(0.1),
                intakeClaw.Close(),
                intakeTurret.setPositionByIndex(2),
                new SleepAction(0.1),
                intakeArm.Transfer(),
                intakePivot.Transfer(),
                extendo.retracted(),
                extendo.waitUntilFinished(),
                intakeArm.waitUntilFinished(),
                extendo.findZero()
        );
    }

    public Action outtakeDeposit() {
        return new SequentialAction(
                t -> {
                    outtakePosition = PositionsClass.OutTake.Deposit;
                    return false;
                },
                outtakePivot.Transfer(),
                outTakeClaw.Close(),
                new SleepAction(0.1),
                intakeClaw.Open(),
                new SleepAction(0.3),
                intakeArm.Extended(),
                intakeArm.waitUntilFinished(),
                new SleepAction(0.1),
                outtakeArm.Deposit(),
                lift.setTargetPosition(1600),
                lift.waitUntilFinished()
        );
    }

    public Action outtakeTransfer() {
        return new SequentialAction(
                t -> {
                    outtakePosition = PositionsClass.OutTake.Transfer;
                    return false;
                },
                outTakeClaw.Open(),
                new SleepAction(0.3),
                outtakePivot.Transfer(),
                outtakeArm.Transfer(),
                lift.transfer(),
                lift.waitUntilFinished()
        );
    }

    public Action outtakeSpecimen(){
        return new SequentialAction(
                t -> {
                    outtakePosition = PositionsClass.OutTake.Specimen;
                    return false;
                },
                outtakePivot.Transfer(),
                outTakeClaw.Close(),
                intakeClaw.Open(),
                new SleepAction(0.3),
                intakeArm.Extended(),
                outtakePivot.Sepcimen(),
                intakeArm.waitUntilFinished(),
                outtakeArm.Deposit(),
                lift.setTargetPosition(550),
                lift.waitUntilFinished()

        );
    }

    public Action depositeSpecimen(){
        return new SequentialAction(
                lift.setTargetPosition(100),
                new SleepAction(0.7),
                outTakeClaw.Open(),
                lift.waitUntilFinished()

        );
    }









    public Action update() {
        return t -> {
            motorControl.update();
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }

        public class Extendo {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.extendo.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.extendo.closeEnough();
                }
            };
        }

        public Action findZero() {
            return new SequentialAction(t -> {motorControl.extendo.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !motorControl.extendo.isResetting()));
        }



        public Action retracted() {
            return setTargetPosition(0);
        }
        public Action extended() {
            return setTargetPosition(590);
        }
    }

    public class Lift {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.lift.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.lift.closeEnough();
                }
            };
        }

        public Action findZero() {
            return new SequentialAction(t -> {motorControl.lift.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !motorControl.lift.isResetting()));
        }



        //todo: fix positions
        public Action transfer() {
            return setTargetPosition(0);
        }
        public Action secondBuceket() {
            return setTargetPosition(500);
        }
        public Action firstTruss() {
            return setTargetPosition(400);
        }
        public Action secondTruss() {
            return setTargetPosition(1500);
        }
    }


    public class IntakeArm {
        private static final double GRAB_POSITION = 0;
        private static final double INTAKE_POSITION = 0.11;
        private static final double EXTENDED_POSITION = 0.25;
        private static final double TRANSFER_POSITION = 0.3;

        public Action setTargetPosition(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.intakeLeft.setTargetPosition(position);
                    motorControl.intakeRight.setPosition(position);
                    return false;
                }
            };
        }

        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.intakeLeft.closeEnough(); // Returns true when both servos are close enough
                }
            };
        }

        public Action Grab() {
            return setTargetPosition(GRAB_POSITION);
        }

        public Action Extended() {
            return setTargetPosition(EXTENDED_POSITION);
        }

        public Action Intake() {
            return setTargetPosition(INTAKE_POSITION);
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }
    }

    public class IntakePivot {

        private static final double GRAB_POSITION = 0;
        private static final double EXTENDED_POSITION = 0;
        private static final double TRANSFER_POSITION = 0.65;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.intakePivot.setPosition(position);
                return false;
            };
        }

        public Action Grab() {
            return setTargetPosition(GRAB_POSITION);
        }

        public Action Extend() {
            return setTargetPosition(EXTENDED_POSITION);
        }

        public Action Transfer(){
            return setTargetPosition(TRANSFER_POSITION);
        }

    }

    public class IntakeTurret {
        // Define positions
        private static final double LEFT2 = 0.2;
        private static final double LEFT1 = 0.35;
        private static final double MIDDLE = 0.5;
        private static final double RIGHT1 = 0.65;
        private static final double RIGHT2 = 0.8;

        private final double[] positions = {LEFT2, LEFT1, MIDDLE, RIGHT1, RIGHT2};
        public int currentPositionIndex = 2; // Start at MIDDLE

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.intakeTurret.setPosition(position);
                return false;
            };
        }

        public Action moveLeft() {
            return t -> {
                if (currentPositionIndex > 0) {
                    currentPositionIndex--;
                    motorControl.intakeTurret.setPosition(positions[currentPositionIndex]);
                }
                return false;
            };
        }

        public Action moveRight() {
            return t -> {
                if (currentPositionIndex < positions.length - 1) {
                    currentPositionIndex++;
                    motorControl.intakeTurret.setPosition(positions[currentPositionIndex]);
                }
                return false;
            };
        }

        public Action setPositionByIndex(int index) {
            return t -> {
                if (index >= 0 && index < positions.length) {
                    currentPositionIndex = index;
                    motorControl.intakeTurret.setPosition(positions[currentPositionIndex]);
                }
                return false;
            };
        }

    }

    public class IntakeClaw {
        private static final double CLOSE_POSITION = 0.05;
        private static final double OPEN_POSITION = 0.35;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.intakeClaw.setPosition(position);
                return false;
            };
        }

        public Action Close() {
            return setTargetPosition(CLOSE_POSITION);
        }

        public Action Open() {
            return setTargetPosition(OPEN_POSITION);
        }
    }

    public class OuttakeArm {

        private static final double TRANSFER_POSITION = 1;
        private static final double DEPOSIT_POSITION = 0.4;
        private static final double SPECIMEN_POSITION = 0.0;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outTakeRight.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Deposit() {
            return setTargetPosition(DEPOSIT_POSITION);
        }

        public Action Sepcimen(){
            return setTargetPosition(SPECIMEN_POSITION);
        }

    }

    public class OuttakePivot {

        private static final double TRANSFER_POSITION = 0.5;
        private static final double DEPOSIT_POSITION = 0.0;
        private static final double SPECIMEN_POSITION = 0.3;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outTakePivot.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Deposit() {
            return setTargetPosition(DEPOSIT_POSITION);
        }

        public Action Sepcimen(){
            return setTargetPosition(SPECIMEN_POSITION);
        }

    }



    public class OutTakeClaw {
        private static final double CLOSE_POSITION = 0.68;
        private static final double OPEN_POSITION = 0.45;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outTakeClaw.setPosition(position);
                return false;
            };
        }

        public Action Close() {
            return setTargetPosition(CLOSE_POSITION);
        }

        public Action Open() {
            return setTargetPosition(OPEN_POSITION);
        }
    }


}
