package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TankDriveLocalizer implements Localizer {
    private final TankDrive tankDrive;
    public final List<Encoder> leftEncs, rightEncs;

    private double lastLeftPos, lastRightPos;
    private boolean initialized;

    public TankDriveLocalizer(TankDrive tankDrive) {
        this.tankDrive = tankDrive;
        {
            List<Encoder> leftEncs = new ArrayList<>();
            for (DcMotorEx m : tankDrive.leftMotors) {
                Encoder e = new OverflowEncoder(new RawEncoder(m));
                leftEncs.add(e);
            }
            this.leftEncs = Collections.unmodifiableList(leftEncs);
        }

        {
            List<Encoder> rightEncs = new ArrayList<>();
            for (DcMotorEx m : tankDrive.rightMotors) {
                Encoder e = new OverflowEncoder(new RawEncoder(m));
                rightEncs.add(e);
            }
            this.rightEncs = Collections.unmodifiableList(rightEncs);
        }

        // TODO: reverse encoder directions if needed
        //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public Pose2d updatePositionEstimate(Pose2d pose) {
        return pose.plus(calculatePoseDelta().value());
    }

    @Override
    public PoseVelocity2d updateVelocityEstimate() {
        return calculatePoseDelta().velocity().value();
    }

    public Twist2dDual<Time> calculatePoseDelta() {
        List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();
        double meanLeftPos = 0.0, meanLeftVel = 0.0;
        for (Encoder e : leftEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            meanLeftPos += p.position;
            meanLeftVel += p.velocity;
            leftReadings.add(p);
        }
        meanLeftPos /= leftEncs.size();
        meanLeftVel /= leftEncs.size();

        double meanRightPos = 0.0, meanRightVel = 0.0;
        for (Encoder e : rightEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            meanRightPos += p.position;
            meanRightVel += p.velocity;
            rightReadings.add(p);
        }
        meanRightPos /= rightEncs.size();
        meanRightVel /= rightEncs.size();

        FlightRecorder.write("TANK_LOCALIZER_INPUTS",
                new TankLocalizerInputsMessage(leftReadings, rightReadings));

        if (!initialized) {
            initialized = true;

            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        TankKinematics.WheelIncrements<Time> twist = new TankKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[]{
                        meanLeftPos - lastLeftPos,
                        meanLeftVel
                }).times(TankDrive.PARAMS.inPerTick),
                new DualNum<Time>(new double[]{
                        meanRightPos - lastRightPos,
                        meanRightVel,
                }).times(TankDrive.PARAMS.inPerTick)
        );

        lastLeftPos = meanLeftPos;
        lastRightPos = meanRightPos;

        return tankDrive.kinematics.forward(twist);
    }
}
