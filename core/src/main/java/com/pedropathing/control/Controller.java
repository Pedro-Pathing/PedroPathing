package com.pedropathing.control;

/**
 * Error Controller.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public interface Controller<C extends Coefficients<C, ? extends Controller<C>>> {
    double run(double error);
    void reset();
    void setCoefficients(C coefficients);
}
