package com.pedropathing.control;

/**
 * A PD controller with quadratic damping to model the non-linear effects of friction.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 * @version after v2.0.0, 9/1/2025
 */
public class QuadraticDampedController implements ErrorController {
    private QuadraticDampedCoefficients coefficients;
    private double error;
    private double velocity;
    
    public QuadraticDampedController(QuadraticDampedCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    
    @Override
    public double run() {
        double predictedBrakingDisplacement =
            velocity * Math.abs(velocity) * coefficients.kQuadraticFriction + velocity * coefficients.kLinearBraking;
        return coefficients.P * (error - predictedBrakingDisplacement) + coefficients.kStaticFriction * Math.signum(velocity);
    }
    
    @Override
    public void updateError(double error) {
        this.error = error;
    }
    
    public void updateVelocity(double velocity) {
        this.velocity = velocity;
    }
    
    @Override
    public QuadraticDampedController setCoefficients(QuadraticDampedCoefficients coefficients) {
        this.coefficients = coefficients;
        return this;
    }
    
    @Override
    public void reset() {
        error = 0;
    }
}
