package com.pedropathing.control;

import java.util.function.Function;

/**
 * This is the PIDFCoefficients class. This class handles holding coefficients for PIDF
 * controllers.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 1.0, 3/5/2024
 */
public class PIDFCoefficients {
    public double P;
    public double I;
    public double D;
    public double F;

    /**
     * This creates a new PIDFCoefficients with constant coefficients.
     *
     * @param p the coefficient for the proportional factor.
     * @param i the coefficient for the integral factor.
     * @param d the coefficient for the derivative factor.
     * @param f the coefficient for the feedforward factor.
     */
    public PIDFCoefficients(double p, double i, double d, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
    }

    /**
     * This returns the coefficient for the feedforward factor.
     *
     * @param input this is inputted into the feedforward equation, if applicable. If there's no
     *              equation, then any input can be used.
     * @return This returns the coefficient for the feedforward factor.
     */
    public double getCoefficient(double input) {
        return F * input;
    }

    public void setCoefficients(double p, double i, double d, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
    }

    @Override
    public String toString() {
        return "P: " + P + ", I: " + I + ", D: " + D + ", F: " + F;
    }
}
