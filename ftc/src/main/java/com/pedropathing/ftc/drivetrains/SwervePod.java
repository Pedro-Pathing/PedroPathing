package com.pedropathing.ftc.drivetrains;

import com.pedropathing.geometry.Pose;

/**
 * Swerve pod interface so Swerve drivetrains can be constructed with coaxial or differential pods.
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public interface SwervePod {

    /**
     * The pod's offset from the robot center (pose offset). It uses the Odometry Coordinate System.
     */
    Pose getOffset();

    /**
     * Returns the pod's current heading (angle after applying the configured offset), in radians.
     *
     * @return heading in radians
     */
    double getAngle();

    /**
     * Convert a wheel-space theta (radians) to the encoder's expected theta/frame.
     *
     * @param wheelTheta wheel-space heading in radians
     * @return encoder-space heading in radians
     */
    double adjustThetaForEncoder(double wheelTheta);

    /**
     * Command the pod to a wheel heading (radians) with a drive power in [-1, 1].
     * If ignoreAngleChanges is true, implementations should avoid applying turn power.
     *
     * @param targetAngleRad desired wheel heading in radians
     * @param drivePower drive power in [0, 1]
     * @param ignoreAngleChanges true to suppress turn power
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
     * Returns a  string useful for debugging the pod state.
     */
    String debugString();
}
