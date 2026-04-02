package com.pedropathing.control;

public class PIDFController implements Controller<PIDFCoefficients> {
    private PIDFCoefficients coefficients;
    private double integral = 0, previousError = 0, previousTime = System.nanoTime();

    public PIDFController(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
        reset();
    }

    public PIDFCoefficients getCoefficients() {
        return coefficients;
    }

    @Override
    public void setCoefficients(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    @Override
    public double calculate(double error) {
        long nanoTime = System.nanoTime();
        double delta = (nanoTime - previousTime);
        previousTime = nanoTime;

        integral += error * (delta / Math.pow(10.0, 9));
        return ((error * coefficients.kP) + (integral * coefficients.kI) + (((error - previousError) / (delta / Math.pow(10.0, 9))) * coefficients.kD) + coefficients.kF);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        previousTime = System.nanoTime();
    }
}
