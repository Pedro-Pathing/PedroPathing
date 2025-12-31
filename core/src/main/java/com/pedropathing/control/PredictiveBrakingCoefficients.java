package com.pedropathing.control;

/**
 * @see PredictiveBrakingController
 * @author Jacob Ophoven - 18535, Frozen Code
 * @version 12/24/2025
 */
public class PredictiveBrakingCoefficients {
    public double kLinearBraking;
    public double kQuadraticFriction;
    public double P;
    public double maximumBrakingPower = 0.2;
    
    /**
     * This creates a new PredictiveBrakingCoefficients with constant coefficients.
     *
     * @param proportional       proportional gain; how strong the robot
     *                           reacts to the remaining error after subtracting
     *                           predicted braking distance. Tune using the LineTest
     *                           after running PredictiveBrakingTuner. Tune as
     *                           high as possible without excessive jitter. Values
     *                           typically range from 0.1 to 0.3.
     * @param linearBraking      Linear braking term (automatically tuned from
     *                           PredictiveBrakingTuner); caused by
     *                           velocity-proportional braking forces. Values typically
     *                           range from 0.15 to 0.05.
     *                           Conceptually similar to a derivative damping term in
     *                           a regular PID.
     * @param quadraticFriction  Quadratic term (automatically tuned from
     *                           PredictiveBrakingTuner); caused by nonlinear friction
     *                           from wheel slip at high speeds.
     */
    public PredictiveBrakingCoefficients(double proportional, double linearBraking, double quadraticFriction) {
        setCoefficients(proportional, linearBraking, quadraticFriction, maximumBrakingPower);
    }
    
    /**
     * Adjusts the maximum amount of power applied in the opposite direction of motion.
     * Too high will cause the control hub to restart due to low voltage spikes. Too low
     * will not have enough power to brake once it slows down after using the motor’s
     * own momentum for braking. Default is 0.2. Must be 0 < x <= 1.
     */
    public PredictiveBrakingCoefficients withMaximumBrakingPower(double maximumBrakingPower) {
        maximumBrakingPower = Math.max(0.0001, Math.min(1.0, maximumBrakingPower));
        setCoefficients(this.P, this.kLinearBraking, this.kQuadraticFriction,
                        maximumBrakingPower);
        return this;
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
