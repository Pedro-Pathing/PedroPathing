package com.pedropathing.control;

/**
 * This is the KalmanFilter class. This creates a Kalman filter that is used to smooth out data.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/17/2024
 */
public class KalmanFilter implements NoiseFilter {
    private KalmanFilterParameters parameters;
    private double state;
    private double variance;
    private double kalmanGain;
    private double previousState;
    private double previousVariance;

    /**
     * This creates a new KalmanFilter from a set of KalmanFilterParameters.
     * @param parameters the parameters to use.
     */
    public KalmanFilter(KalmanFilterParameters parameters) {
        this.parameters = parameters;
        reset();
    }

    /**
     * This creates a new KalmanFilter from a set of KalmanFilterParameters, a starting state,
     * a starting variance, and a starting Kalman gain.
     *
     * @param parameters   the parameters to use.
     * @param startState   the starting state.
     * @param startVariance the starting variance.
     * @param startGain    the starting Kalman gain.
     */
    public KalmanFilter(KalmanFilterParameters parameters, double startState, double startVariance, double startGain) {
        this.parameters = parameters;
        reset(startState, startVariance, startGain);
    }

    public void reset(double startState, double startVariance, double startGain) {
        state = startState;
        previousState = startState;
        variance = startVariance;
        previousVariance = startVariance;
        kalmanGain = startGain;
    }

    public void reset() {
        reset(0, 1, 1);
    }

    public void update(double updateData, double updateProjection) {
        state = previousState + updateData;
        variance = previousVariance + parameters.modelCovariance;
        kalmanGain = variance / (variance + parameters.dataCovariance);
        state += kalmanGain * (updateProjection - state);
        variance *= (1.0 - kalmanGain);
        previousState = state;
        previousVariance = variance;
    }

    public double getState() {
        return state;
    }

    /**
     * This method outputs the current state, variance, and Kalman gain of the filter as a string array.
     * @return A string array containing the current state, variance, and Kalman gain.
     */
    public String[] output() {
        return new String[]{
                "State: " + state,
                "Variance: " + variance,
                "Kalman Gain: " + kalmanGain
        };
    }
}
