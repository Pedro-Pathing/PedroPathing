package com.pedropathing.ftc.drivetrains;

import com.pedropathing.geometry.Pose;

/**
 * Swerve pod interface that abstracts non-hardware-specific behavior of a swerve pod.
 * Implementations live in platform-specific modules and own the hardware (motors, servos,
 * encoders, controllers).
 * @author Baron Henderson
 */
public interface SwervePod {

    /**
     * The pod's offset from the robot center (pose offset). It uses the Odometry Coordinate System.
     */
    Pose getOffset();

    /**
     * Returns the pod's current heading (angle after applying the configured offset), in radians.
     */
    double getAngle();

    /**
     * Convert a wheel-space theta (radians) to the encoder's expected theta/frame. This
     * encapsulates encoder orientation handling so callers don't need to branch.
     */
    double adjustThetaForEncoder(double wheelTheta);

    /**
     * Command the pod to a wheel heading (radians) with a drive power in [-1, 1].
     * Implementations own any hardware-specific thresholds/flags and pull those from
     * internal state; the interface exposes only the logical command inputs.
     */
    void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges);

    /**
     * Set drivetrain hardware to float (zero power behaviour FLOAT) for the pod's drive motor.
     */
    void setToFloat();

    /**
     * Set drivetrain hardware to brake (zero power behaviour BRAKE) for the pod's drive motor.
     */
    void setToBreak();

    /**
     * Returns a compact string useful for debugging the pod state.
     */
    String debugString();
}
