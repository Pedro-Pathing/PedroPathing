/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.controllers.filters;

/**
 * This is the KalmanFilter class, used to fuse multiple inputs.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 2.0, 5/25/2026
 */
public class KalmanFilter {
    private final double modelCovariance;
    private final double dataCovariance;
    private double state;
    private double variance;
    private double kalmanGain;

    /**
     * This creates a new KalmanFilter.
     * @param modelCovariance the model's covariance, describing the randomness in the system.
     * @param dataCovariance the data's covariance, describing uncertainty in sensor outputs
     */
    public KalmanFilter(double modelCovariance, double dataCovariance) {
        this.modelCovariance = modelCovariance;
        this.dataCovariance = dataCovariance;
        reset();
    }

    /**
     * This creates a new KalmanFilter from a starting state,
     * a starting variance, and a starting Kalman gain.
     * @param modelCovariance the model's covariance, describing the randomness in the system.
     * @param dataCovariance the data's covariance, describing uncertainty in sensor outputs
     * @param startState   the starting state.
     * @param startVariance the starting variance.
     * @param startGain    the starting Kalman gain.
     */
    public KalmanFilter(
            double modelCovariance, double dataCovariance, double startState, double startVariance, double startGain) {
        this.modelCovariance = modelCovariance;
        this.dataCovariance = dataCovariance;
        reset(startState, startVariance, startGain);
    }

    public void reset(double startState, double startVariance, double startGain) {
        state = startState;
        variance = startVariance;
        kalmanGain = startGain;
    }

    public void reset() {
        reset(0, 1, 1);
    }

    public void update(double updateData, double updateProjection) {
        state += updateData;
        variance += modelCovariance;
        kalmanGain = variance / (variance + dataCovariance);
        state += kalmanGain * (updateProjection - state);
        variance *= (1.0 - kalmanGain);
    }

    /**
     * Use this for a single-input Kalman Filter update
     * @param measurement the new measured value
     */
    public void update(double measurement) {
        update(0, measurement);
    }

    public double state() {
        return state;
    }

    /**
     * This method outputs the current state, variance, and Kalman gain of the filter as a string array.
     * @return A string array containing the current state, variance, and Kalman gain.
     */
    public String[] output() {
        return new String[] {"State: " + state, "Variance: " + variance, "Kalman Gain: " + kalmanGain};
    }
}
