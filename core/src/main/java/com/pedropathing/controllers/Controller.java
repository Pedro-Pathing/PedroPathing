/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.controllers;

@FunctionalInterface
public interface Controller {
    double calculate(double target, double error);

    Controller zero = (t, e) -> 0;

    default double calculate(double target, double error, double velocity) {
        return calculate(target, error);
    }

    default void reset() {}

    static Controller staticFeedforward(double kStatic) {
        return (t, e) -> kStatic * Math.signum(e);
    }

    static Controller dynamicFeedforward(double kF) {
        return (t, e) -> t * kF;
    }

    static PIDController pid(double kP, double kI, double kD) {
        return new PIDController(kP, kI, kD);
    }

    static PiecewiseController piecewise(Controller baseline) {
        return new PiecewiseController(baseline);
    }

    default Controller plus(Controller other) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                return Controller.this.calculate(target, error) + other.calculate(target, error);
            }

            @Override
            public void reset() {
                Controller.this.reset();
                other.reset();
            }
        };
    }

    default Controller minus(Controller other) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                return Controller.this.calculate(target, error) - other.calculate(target, error);
            }

            @Override
            public void reset() {
                Controller.this.reset();
                other.reset();
            }
        };
    }

    default Controller times(double scalar) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                return Controller.this.calculate(target, error) * scalar;
            }

            @Override
            public void reset() {
                Controller.this.reset();
            }
        };
    }
}
