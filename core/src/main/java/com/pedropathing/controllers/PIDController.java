/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.controllers;

public class PIDController implements Controller {
    public final double kP, kI, kD;
    private double integral = 0, previousError = 0, previousTime = System.nanoTime();

    PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    public double calculate(double target, double error) {
        long nanoTime = System.nanoTime();
        double delta = nanoTime - previousTime;
        previousTime = nanoTime;

        integral += error * (delta / Math.pow(10.0, 9));
        return (error * kP) + (integral * kI) + (((error - previousError) / (delta / Math.pow(10.0, 9))) * kD);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        previousTime = System.nanoTime();
    }
}
