package com.pedropathing.control;

/**
 * Error Controller.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public interface Controller {
    double run(double error);
    void reset();
}
