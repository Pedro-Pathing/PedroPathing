package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

/**
 * This is the PoseTracker class. This class handles getting pose data from the localizer and returning
 * the information in a useful way to the Follower.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/4/2024
 */
public class PoseTracker {

    private final Localizer localizer;

    private Pose startingPose = new Pose(0,0,0);

    private Pose currentPose = startingPose.copy();

    private Pose previousPose = startingPose.copy();

    private Pose currentVelocity = new Pose();

    private Vector previousVelocity = new Vector();

    private Vector currentAcceleration = new Vector();

    private double xOffset = 0;
    private double yOffset = 0;
    private double headingOffset = 0;

    private long previousPoseTime;
    private long currentPoseTime;

    /**
     * Creates a new PoseTracker from a Localizer.
     *
     * @param localizer the Localizer
     */
    public PoseTracker(Localizer localizer) {
        this.localizer = localizer;

        try {
            localizer.resetIMU();
        } catch (InterruptedException ignored) {
            System.out.println("PoseTracker: resetIMU() interrupted");
        }
    }


    /**
     * This updates the robot's pose, as well as updating the previous pose, velocity, and
     * acceleration. The cache for the current pose, velocity, and acceleration is cleared, and
     * the time stamps are updated as well.
     */
    public void update() {
        previousVelocity = getVelocity();
        previousPose = applyOffset(getRawPose());
        currentPose = null;
        currentVelocity = null;
        currentAcceleration = null;
        previousPoseTime = currentPoseTime;
        currentPoseTime = System.nanoTime();
        localizer.update();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param set the Pose to set the starting pose to.
     */
    public void setStartingPose(Pose set) {
        startingPose = set;
        previousPose = startingPose;
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
        localizer.setStartPose(set);
    }

    /**
     * This sets the current pose, using offsets. Think of using offsets as setting trim in an
     * aircraft. This can be reset as well, so beware of using the resetOffset() method.
     *
     *
     * @param set The pose to set the current pose to.
     */
    public void setCurrentPoseWithOffset(Pose set) {
        Pose currentPose = getRawPose();
        setXOffset(set.getX() - currentPose.getX());
        setYOffset(set.getY() - currentPose.getY());
        setHeadingOffset(MathFunctions.getTurnDirection(currentPose.getHeading(), set.getHeading()) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), set.getHeading()));
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param offset This sets the offset.
     */
    public void setXOffset(double offset) {
        xOffset = offset;
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param offset This sets the offset.
     */
    public void setYOffset(double offset) {
        yOffset = offset;
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param offset This sets the offset.
     */
    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    public double getXOffset() {
        return xOffset;
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    public double getYOffset() {
        return yOffset;
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    public double getHeadingOffset() {
        return headingOffset;
    }

    /**
     * This applies the offset to a specified Pose.
     *
     * @param pose The pose to be offset.
     * @return This returns a new Pose with the offset applied.
     */
    public Pose applyOffset(Pose pose) {
        return new Pose(pose.getX()+xOffset, pose.getY()+yOffset, pose.getHeading()+headingOffset);
    }

    /**
     * This resets all offsets set to the PoseTracker. If you have reset your pose using the
     * setCurrentPoseUsingOffset(Pose2d set) method, then your pose will be returned to what the
     * PoseTracker thinks your pose would be, not the pose you reset to.
     */
    public void resetOffset() {
        setXOffset(0);
        setYOffset(0);
        setHeadingOffset(0);
    }

    /**
     * This returns the current pose, with offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the current pose.
     */
    public Pose getPose() {
        if (currentPose == null) {
            currentPose = localizer.getPose();
        }
        return applyOffset(currentPose);
    }

    /**
     * This returns the current raw pose, without any offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the raw pose.
     */
    public Pose getRawPose() {
        if (currentPose == null) {
            currentPose = localizer.getPose();
        }
        return currentPose;
    }

    /**
     * This sets the current pose without using resettable offsets.
     *
     * @param set the pose to set the current pose to.
     */
    public void setPose(Pose set) {
        resetOffset();
        localizer.setPose(set);
    }

    /**
     * Returns the robot's pose from the previous update.
     *
     * @return returns the robot's previous pose.
     */
    public Pose getPreviousPose() {
        return previousPose;
    }

    /**
     * Returns the robot's change in pose from the previous update.
     *
     * @return returns the robot's delta pose.
     */
    public Pose getDeltaPose() {
        Pose returnPose = getPose();
        return returnPose.minus(previousPose);
    }

    /**
     * This returns the velocity of the robot as a Vector. If this is called multiple times in
     * a single update, the velocity Vector is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the velocity of the robot.
     */
    public Vector getVelocity() {
        if (currentVelocity == null) currentVelocity = localizer.getVelocity();
        return currentVelocity.getAsVector();
    }

    /**
     * This returns the angular velocity of the robot as a double.
     *
     * @return returns the angular velocity of the robot.
     */
    public double getAngularVelocity() {
        if (currentVelocity == null) currentVelocity = localizer.getVelocity();
        return currentVelocity.getHeading();
    }

    /**
     * This returns the acceleration of the robot as a Vector. If this is called multiple times in
     * a single update, the acceleration Vector is cached so that subsequent calls don't have to
     * repeat localizer calls or calculations.
     *
     * @return returns the acceleration of the robot.
     */
    public Vector getAcceleration() {
        if (currentAcceleration == null) {
            currentAcceleration = getVelocity().minus(previousVelocity);
            currentAcceleration.setMagnitude(currentAcceleration.getMagnitude() / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9)));
        }
        return currentAcceleration.copy();
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using Road Runner's pose reset.
     */
    public void resetHeadingToIMU() {
        if (!Double.isNaN(getIMUHeadingEstimate()) && !Double.isInfinite(getIMUHeadingEstimate())) {
            localizer.setPose(new Pose(getPose().getX(), getPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using offsets instead of Road
     * Runner's pose reset. This way, it's faster, but this can be wiped with the resetOffsets()
     * method.
     */
    public void resetHeadingToIMUWithOffsets() {
        if (!Double.isNaN(getIMUHeadingEstimate()) && !Double.isInfinite(getIMUHeadingEstimate())) {
            setCurrentPoseWithOffset(new Pose(getPose().getX(), getPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians.
     *
     * @return returns the normalized IMU heading.
     */
    public double getNormalizedIMUHeading() {
        if (!Double.isNaN(getIMUHeadingEstimate()) && !Double.isInfinite(getIMUHeadingEstimate())) {
            return MathFunctions.normalizeAngle(-getIMUHeadingEstimate());
        }
        return 0;
    }

    /**
     * This returns the total number of radians the robot has turned.
     *
     * @return the total heading.
     */
    public double getTotalHeading() {
        return localizer.getTotalHeading();
    }

    /**
     * This returns the Localizer.
     *
     * @return the Localizer
     */
    public Localizer getLocalizer() {
        return localizer;
    }

    /**
     * This returns the IMU heading.
     *
     * @return the IMU heading
     */
    public double getIMUHeadingEstimate() {
        return localizer.getIMUHeading();
    }

    /**
     * This resets the IMU of the localizer.
     */
    public void resetIMU() throws InterruptedException {
        localizer.resetIMU();
    }

    public String debugString() {
        return "PoseTracker{" +
                "currentPose=" + getPose() +
                ", previousPose=" + getPreviousPose() +
                ", currentVelocity=" + getVelocity() +
                ", previousVelocity=" + previousVelocity +
                ", currentAcceleration=" + getAcceleration() +
                ", xOffset=" + getXOffset() +
                ", yOffset=" + getYOffset() +
                ", headingOffset=" + getHeadingOffset() +
                '}';
    }
}
