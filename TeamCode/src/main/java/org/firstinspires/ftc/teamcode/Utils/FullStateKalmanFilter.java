package org.firstinspires.ftc.teamcode.Utils;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionState;

/**
 * from position this kalman filter will estimate the velocity and acceleration states.
 */
public class FullStateKalmanFilter {


    ElapsedTime timer = new ElapsedTime();

    protected boolean hasRun = false;

    protected double dt = 1000/100.0;

    private SimpleMatrix A, C, Q,R,K,K_prev, P, P_prev, state_estimate;

    protected MotionState state;

    public FullStateKalmanFilter(double initialPosition, double initialVelocity, double initialAcceleration) {
        this.state = new MotionState(initialPosition,initialVelocity,initialAcceleration);
        // the only observable state is position, so it gets a 1

        C = new SimpleMatrix(new double[][] {{1,0,0}});
    }

    public FullStateKalmanFilter(double initialPosition) {
        this(initialPosition,0,0);
    }
    public FullStateKalmanFilter(double initialPosition, double initialVelocity) {
        this(initialPosition,initialVelocity,0);
    }


    public void updateKalmanFilter(double position) {
        calculateDT();

    }

    protected void calculateDT() {
        if (!hasRun) {
            timer.reset();
            hasRun = false;
        }
        dt = timer.seconds();
        timer.reset();
    }

    protected void setMatrices() {

        // A matrix
        A = new SimpleMatrix(new double[][]{{1, dt, 0.5 * dt * dt},
                                            {0,1,dt},
                                            {0,0,1}});
        // Q Matrix
//        Q = np.array([[dt ** 4 / 4,dt ** 3 / 2,dt ** 2 / 2]
////             ,[dt ** 3 / 2,dt ** 2,dt]
////             ,[dt**2/2,dt,1]]) * 0.1

        Q = new SimpleMatrix(new double[][] {
                {Math.pow(dt,4) / 4, Math.pow(dt,3) / 2,Math.pow(dt,2) / 2},
                {Math.pow(dt,3) / 2, Math.pow(dt,2),dt},
                {Math.pow(dt,2) * 2, dt, 1}
        });

        // R Matrix
//        R = np.eye(1) * 1e5


    }



    public MotionState getState() {
        return state;
    }


}
