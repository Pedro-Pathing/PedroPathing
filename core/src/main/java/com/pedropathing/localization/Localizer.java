package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;

import com.pedropathing.math.Vector;

/**
 * This is the Localizer class. It is an abstract superclass of all localizers used in Pedro Pathing,
 * so it contains abstract methods that will have a concrete implementation in the subclasses. Any
 * method that all localizers will need will be in this class.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public interface Localizer {

    /**
     * This returns the current pose estimate from the Localizer.
     *
     * @return returns the pose as a Pose object.
     */
    Pose getPose();

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose object.
     */
    Pose getVelocity();

    /**
     * This returns the current velocity estimate from the Localizer as a Vector.
     *
     * @return returns the velocity as a Vector.
     */
    Vector getVelocityVector();

    /**
     * This sets the start pose of the Localizer. Changing the start pose should move the robot as if
     * all its previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    void setStartPose(Pose setStart);

    /**
     * This sets the current pose estimate of the Localizer. Changing this should just change the
     * robot's current pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    void setPose(Pose setPose);

    /**
     * This calls an update to the Localizer, updating the current pose estimate and current velocity
     * estimate.
     */
    void update();

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    double getTotalHeading();

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    double getForwardMultiplier();

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    double getLateralMultiplier();

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    double getTurningMultiplier();

    /**
     * This resets the IMU of the localizer, if applicable.
     */
    void resetIMU() throws InterruptedException;

    /**
     * This is overridden to return the IMU's heading estimate, if there is one.
     *
     * @return returns the IMU's heading estimate if it exists
     */
    double getIMUHeading();

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns if any component of the robot's position is NaN
     */
    boolean isNAN();

    /**
     * This sets the current x-position estimate of the localizer. Units are inferred from localizer constants where necessary.
     * @param x the x-position estimate to set
     */
    default void setX(double x) {
        setPose(getPose().withX(x));
    }

    /**
     * This sets the current y-position estimate of the localizer. Units are inferred from localizer constants where necessary.
     * @param y the y-position estimate to set
     */
    default void setY(double y) {
        setPose(getPose().withY(y));
    }

    /**
     * This sets the current heading estimate of the localizer, in radians.
     * @param heading the heading estimate to set
     */
    default void setHeading(double heading) {
        setPose(getPose().withHeading(heading));
    }
}
