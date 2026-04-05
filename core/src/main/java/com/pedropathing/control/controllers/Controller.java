package com.pedropathing.control.controllers;

@FunctionalInterface
public interface Controller {
    double calculate(double target, double error);
    default void reset() {};

    static Controller staticFeedforward(double kStatic) {
        return (t,e) -> kStatic * Math.signum(e);
    }

    static Controller dynamicFeedforward(double kF) {
        return (t, e) -> t * kF;
    }

    static PIDController pid(PIDCoefficients coefficients) {
        return new PIDController(coefficients);
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
}
