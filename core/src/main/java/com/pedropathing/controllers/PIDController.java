/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.controllers;

public class PIDController implements Controller {
    public final PIDCoefficients coefficients;
    private double integral = 0, previousError = 0, previousTime = System.nanoTime();

    PIDController(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    @Override
    public double calculate(double target, double error) {
        long nanoTime = System.nanoTime();
        double delta = nanoTime - previousTime;
        previousTime = nanoTime;

        integral += error * (delta / Math.pow(10.0, 9));
        return (error * coefficients.kP)
                + (integral * coefficients.kI)
                + (((error - previousError) / (delta / Math.pow(10.0, 9))) * coefficients.kD);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        previousTime = System.nanoTime();
    }
}
