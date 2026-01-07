package com.pedropathing.control;

/**
 * A positional controller that maximizes deceleration by measuring the robot’s braking
 * behavior rather than assuming it.
 * <p>
 * Instead of relying on a manually tuned derivative term to prevent overshoot, this
 * controller predicts how far the robot will slide if it brakes using zero-power brake
 * mode. It uses the predicted braking distance to anticipate positional error,
 * effectively treating it as reaction time. This allows the robot to brake precisely
 * when needed, maximizing deceleration, responsiveness, and accuracy.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 * @version 12/24/2025
 * @see PredictiveBrakingCoefficients
 */
public class PredictiveBrakingController {
    private PredictiveBrakingCoefficients coefficients;
    
    public PredictiveBrakingController(PredictiveBrakingCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    
    public void setCoefficients(PredictiveBrakingCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    
    /**
     * Computes the control output.
     * <p>
     * Predicts braking displacement using: velocity * |velocity| * kFriction   //
     * quadratic slippage + velocity * kBraking                 // linear braking
     * <p>
     * Then subtracts this prediction from the error and scales by kP.
     * <p>
     *
     * @param error    current positional error
     * @param velocity current velocity
     * @return control output (e.g., motor power)
     */
    public double computeOutput(double error, double velocity) {
        double directionOfMotion = Math.signum(velocity);
        
        double outputPower =
            coefficients.P * (error - computeBrakingDisplacement(velocity, directionOfMotion));
        
        return clampReversePower(outputPower, directionOfMotion);
    }
    
    public double computeBrakingDisplacement(double velocity, double directionOfMotion) {
        return directionOfMotion * velocity * velocity * coefficients.kQuadraticFriction
                + velocity * coefficients.kLinearBraking;
    }
    
    /**
     * Prevents the controller from applying too much power in the opposite direction of
     * the robot's momentum. Alternating full forward (+1) and full reverse (-1) power
     * caused the control hub to restart due to low voltage spikes. This fixes it by
     * capping the amount of voltage applied opposite to the direction of motion to be
     * very minimal. Even a tiny opposite voltage (e.g., -0.0001) locks the wheels like
     * zero-power brake mode, using the motor’s own momentum for braking without consuming
     * significant energy.
     */
    private double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        if (!isOpposingMotion) {
            return power;
        }
        double clampedPower;
        if (power < 0) {
            clampedPower = Math.max(power, -coefficients.maximumBrakingPower);
        } else {
            clampedPower = Math.min(power, coefficients.maximumBrakingPower);
        }
        return clampedPower;
    }
}
