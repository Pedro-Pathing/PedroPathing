package com.pedropathing.control;

/**
 * This is the CustomFilteredPIDFCoefficients class. This class handles holding coefficients for filtered PIDF
 * controllers.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 * @version 12/24/2025
 */
public class PredictiveBrakingCoefficients {
    /**
     * Linear braking coefficient. Models speed-proportional braking force (how much force
     * the robot brakes with relative to it's velocity). Conceptually similar to a
     * derivative damping term in a regular PID.
     */
    public double kLinearBraking;
    
    /**
     * Quadratic friction/slippage coefficient. Models non-linear wheel slippage from
     * friction.
     */
    public double kQuadraticFriction;
    
    /**
     * Proportional gain. Controls how strongly the robot reacts to the remaining error
     * after subtracting predicted braking displacement.
     */
    public double P;
    
    /**
     * Maximum amount of power applied in the opposite direction of momentum. To high will
     * cause the control hub to restart due to low voltage spikes. Too low will not have
     * enough power to brake once it slows down after using the motor’s own momentum for
     * braking.
     */
    public double maximumBrakingPower = 0.2;
    
    /**
     * This creates a new CustomFilteredPIDFCoefficients with constant coefficients.
     *
     * @param p                  proportional gain
     * @param linearBraking      linear damping coefficient
     * @param quadraticFriction  quadratic slippage/friction coefficient
     * @param maximumBrakingPower maximum braking power applied in opposite direction
     *                            of momentum (0.2 is a good starting point) Must be >
     *                            0 and <= 1.
     */
    public PredictiveBrakingCoefficients(double p, double linearBraking, double quadraticFriction,
                                         double maximumBrakingPower) {
        setCoefficients(p, linearBraking, quadraticFriction, maximumBrakingPower);
    }
    
    public void setCoefficients(double p, double linearBraking, double quadraticFriction,
                                double maximumBrakingPower) {
        this.P = p;
        this.kLinearBraking = linearBraking;
        this.kQuadraticFriction = quadraticFriction;
        this.maximumBrakingPower = maximumBrakingPower;
    }
    
    @Override
    public String toString() {
        return "P: " + P + ", L: " + kLinearBraking + ", Q: " + kQuadraticFriction;
    }
}
