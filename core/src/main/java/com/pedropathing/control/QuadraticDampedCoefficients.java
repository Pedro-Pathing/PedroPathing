package com.pedropathing.control;

/**
 * A PD controller with quadratic damping to model the non-linear effects of friction
 * when braking.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 * @version after v2.0.0, 9/1/2025
 */
public class QuadraticDampedCoefficients implements ControllerCoefficients {
    public double P;
    public double kLinearBraking;
    public double kQuadraticFriction;
    public double kStaticFriction;
    
    /**
     * @param p the proportional gain
     * @param kLinearBraking the linear damping coefficient (also can be though of
     *                       as the derivative gain)
     * @param kQuadraticFriction the quadratic friction coefficient to account for
     *                           non-linear friction effects when braking
     * @param kStaticFriction the static friction coefficient to just not overcome
     *                        initial friction so it can stop.
     */
    public QuadraticDampedCoefficients(double p,
                                       double kLinearBraking,
                                       double kQuadraticFriction,
                                       double kStaticFriction) {
        this.P = p;
        this.kLinearBraking = kLinearBraking;
        this.kQuadraticFriction = kQuadraticFriction;
        this.kStaticFriction = kStaticFriction;
    }
    
    @Override
    public String toString() {
        return "P: " + P + ", D: " + kLinearBraking + ", Q: " + kQuadraticFriction;
    }
    
    @Override
    public QuadraticDampedController create() {
        return new QuadraticDampedController(this);
    }
}
