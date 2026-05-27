package com.pedropathing.controllers;

import com.pedropathing.controllers.filters.KalmanFilter;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class KalmanFilterTest {
    @Test
    public void updateReducesVarianceAndMovesStateTowardMeasurement() {
        KalmanFilter kf = new KalmanFilter(1.0, 1.0);
        // initial state 0, variance 1
        kf.update(10.0);
        double state = kf.state();
        String[] out = kf.output();
        assertTrue(state > 0.0 && state < 10.0);
        assertTrue(out[0].contains("State:"));
        assertTrue(out[1].contains("Variance:"));
        assertTrue(out[2].contains("Kalman Gain:"));

        // Known expected values for this configuration
        assertEquals(20.0/3.0, state, 1e-9); // computed as 6.666...
    }
}

