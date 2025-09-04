package com.pedropathing.control;

/**
 * A PD controller with quadratic damping to model the non-linear effects of friction.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 * @version after v2.0.0, 9/1/2025
 */
public class QuadraticDampedController implements Controller {
    private QuadraticDampedCoefficients coefficients;
    private double previousError;

    private long previousUpdateTimeNano;
    
    public QuadraticDampedController(QuadraticDampedCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    
    @Override
    public double run(double error) {
        double derivative = (error - previousError) / ((System.nanoTime() - previousUpdateTimeNano) / 1e9);
        double velocity = -derivative;
        previousUpdateTimeNano = System.nanoTime();
        previousError = error;
        
        double predictedBrakingDisplacement =
            velocity * Math.abs(velocity) * coefficients.kQuadraticFriction + velocity * coefficients.kLinearBraking;
        return coefficients.P * (error - predictedBrakingDisplacement) + coefficients.kStaticFriction * Math.signum(velocity);
    }
    
    public void setCoefficients(QuadraticDampedCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    
    @Override
    public void reset() {
        previousError = 0;
    }
}
