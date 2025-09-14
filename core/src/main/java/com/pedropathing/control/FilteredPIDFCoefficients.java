package com.pedropathing.control;

/**
 * This is the CustomFilteredPIDFCoefficients class. This class handles holding coefficients for filtered PIDF
 * controllers.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/15/2024
 */

public class FilteredPIDFCoefficients {
    public double P;
    public double I;
    public double D;
    public double F;
    public double T;

    /**
     * This creates a new CustomFilteredPIDFCoefficients with constant coefficients.
     *
     * @param p the coefficient for the proportional factor.
     * @param i the coefficient for the integral factor.
     * @param d the coefficient for the derivative factor.
     * @param t the time constant for the filter
     * @param f the coefficient for the feedforward factor.
     */
    public FilteredPIDFCoefficients(double p, double i, double d, double t, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
        T = t;
    }

    public void setCoefficients(double p, double i, double d, double t, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
        T = t;
    }

    @Override
    public String toString() {
        return "P: " + P + ", I: " + I + ", D: " + D + ", T: " + T + ", F: " + F;
    }
}
