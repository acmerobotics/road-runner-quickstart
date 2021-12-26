package org.firstinspires.ftc.teamcode.util.KalmanFilter;


import android.os.Build;

import androidx.annotation.RequiresApi;

public class LeastSquaresKalmanFilter {

    protected double Q;
    protected double R;
    protected int N;
    protected double P = 1;
    protected double K = 0;
    protected double x;
    protected SizedStack<Double> estimates;
    protected LinearRegression regression;

    /**
     * A kalman filter that uses a least squares regression as it's model.
     * @param Q Sensor Noise Covariance
     * @param R Model Covariance
     * @param N Number of elements we can hold in our stack.
     */
    public LeastSquaresKalmanFilter(double Q, double R, int N) {
        this.Q = Q;
        this.R = R;
        this.N = N;
        this.x = 0;
        this.estimates = new SizedStack<>(N);
        initializeStackWith0();
        regression = new LinearRegression(stackToDoubleArray());
        solveK();
    }

    /**
     * update the kalman filter for traditional; continous values.
     * @param measurement the current measurement
     * @return the optimal state estimate.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public double update(double measurement) {
        regression.runLeastSquares();
        x += regression.predictNextValue() - estimates.peek();
        x += K * (measurement - x);
        estimates.push(x);
        regression = new LinearRegression(stackToDoubleArray());
        return x;
    }


    /**
     * Iteratively compute K using the D.A.R.E
     */
    public void solveK() {
        for (int i = 0; i < 200; ++i) solveDARE();
    }

    /**
     * solve the discrete time algebraic riccati equation (D.A.R.E)
     */
    public void solveDARE() {
        P = P + Q;
        K = P / (P + R);
        P = (1-K) * P;
    }

    /**
     * initialize the stack to all 0's
     */
    protected void initializeStackWith0() {
        for (int i = 0; i < N; ++i) {
            estimates.push(0.0);
        }
    }

    /**
     * convert the stack to an array of doubles
     * @return an array of doubles.
     */
    protected double[] stackToDoubleArray() {
        double[] newValues = new double[N];
        for (int i = 0; i < estimates.size(); ++i) {
            newValues[i] = estimates.get(i);
        }
        return  newValues;
    }

}
