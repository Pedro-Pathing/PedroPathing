package com.pedropathing.control;

/**
 * This is the SquIDCoefficients class. This class holds the coefficients for a SquID controller.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class SquIDCoefficients implements Controller.Coefficients {
    private double P;
    private double F;

    @Override
    public Controller create() {
        return new SquIDController(this);
    }

    @Override
    public void setCoefficients(double... coefficients) {
        if (coefficients.length != 2) {
            throw new IllegalArgumentException("Expected 1 coefficient, got " + coefficients.length);
        }
        P = coefficients[0];
        F = coefficients[1];
    }

    public double getP() {
        return P;
    }

    public void setP(double p) {
        P = p;
    }

    public double getF() {
        return F;
    }

    public void setF(double f) {
        F = f;
    }
}
