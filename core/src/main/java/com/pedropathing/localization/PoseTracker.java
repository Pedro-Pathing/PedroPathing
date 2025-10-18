package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

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


    private final double kBufferDuration = 1.5;

    private final NavigableMap<Double, Pose> m_odometryPoseBuffer = new TreeMap<>();

    private final NavigableMap<Double, PoseTracker.VisionUpdate> m_visionUpdates = new TreeMap<>();

    private final double[] m_odometryStdDevs = new double[] {0.05, 0.05, 0.02};
    private final double[] m_visionMeasurementStdDevs = new double[] {0.5, 0.5, 0.2};

    private final double[] m_q = new double[3];
    private final double[][] m_visionK = new double[3][3];


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

        setOdometryStdDevs(m_odometryStdDevs[0], m_odometryStdDevs[1], m_odometryStdDevs[2]);
        setVisionMeasurementStdDevs(m_visionMeasurementStdDevs[0], m_visionMeasurementStdDevs[1], m_visionMeasurementStdDevs[2]);

        currentPose = localizer.getPose().copy();
        previousPose = localizer.getPose().copy();
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
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

        // sample and store odometry-only pose at this timestamp (seconds)
        double nowSeconds = System.nanoTime() / 1e9;
        Pose odomPose = localizer.getPose().copy();
        m_odometryPoseBuffer.put(nowSeconds, odomPose);

        // clean up old odometry samples
        cleanUpOdometryBuffer(nowSeconds);

        // update current pose estimate by applying the latest vision compensation (if any)
        if (m_visionUpdates.isEmpty()) {
            currentPose = odomPose.copy();
        } else {
            PoseTracker.VisionUpdate latestVU = m_visionUpdates.get(m_visionUpdates.lastKey());
            currentPose = latestVU.compensate(odomPose);
        }
    }

    /**
     * Set the standard deviations (x, y, heading) for the odometry uncertainty.
     */
    public void setOdometryStdDevs(double xStd, double yStd, double headingStd) {
        m_odometryStdDevs[0] = xStd;
        m_odometryStdDevs[1] = yStd;
        m_odometryStdDevs[2] = headingStd;
        for (int i = 0; i < 3; ++i) {
            m_q[i] = m_odometryStdDevs[i] * m_odometryStdDevs[i];
        }
        recomputeVisionK();
    }

    /**
     * Set the standard deviations (x, y, heading) for vision measurement uncertainty.
     */
    public void setVisionMeasurementStdDevs(double xStd, double yStd, double headingStd) {
        m_visionMeasurementStdDevs[0] = xStd;
        m_visionMeasurementStdDevs[1] = yStd;
        m_visionMeasurementStdDevs[2] = headingStd;
        recomputeVisionK();
    }

    private void recomputeVisionK() {
        // r = vision variance
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = m_visionMeasurementStdDevs[i] * m_visionMeasurementStdDevs[i];
        }

        // zero the matrix first (ensures off-diagonals are cleared)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m_visionK[i][j] = 0.0;
            }
        }

        // closed-form diagonal Kalman-like gain used by WPILib:
        // k = q / ( q + sqrt(q * r) ), placed on the diagonal
        for (int i = 0; i < 3; ++i) {
            double k;
            if (m_q[i] == 0.0) {
                k = 0.0;
            } else {
                k = m_q[i] / (m_q[i] + Math.sqrt(m_q[i] * r[i]));
            }
            m_visionK[i][i] = k;
        }
    }

    private void cleanUpOdometryBuffer(double nowSeconds) {
        double cutoff = nowSeconds - kBufferDuration;
        m_odometryPoseBuffer.headMap(cutoff, false).clear();
        cleanUpVisionUpdates();
    }

    /**
     * Return the odometry-only pose at given timestamp (seconds) by linear interpolation.
     * Returns null if buffer empty.
     */
    public Pose getOdometrySample(double timestampSeconds) {
        if (m_odometryPoseBuffer.isEmpty()) {
            return null;
        }

        double oldest = m_odometryPoseBuffer.firstKey();
        double newest = m_odometryPoseBuffer.lastKey();

        // clamp
        timestampSeconds = Math.max(oldest, Math.min(newest, timestampSeconds));

        if (m_odometryPoseBuffer.containsKey(timestampSeconds)) {
            return m_odometryPoseBuffer.get(timestampSeconds).copy();
        }

        Map.Entry<Double, Pose> floor = m_odometryPoseBuffer.floorEntry(timestampSeconds);
        Map.Entry<Double, Pose> ceil = m_odometryPoseBuffer.ceilingEntry(timestampSeconds);

        if (floor == null) return ceil.getValue().copy();
        if (ceil == null) return floor.getValue().copy();

        if (floor.getKey().equals(ceil.getKey())) {
            return floor.getValue().copy();
        }

        double t0 = floor.getKey();
        double t1 = ceil.getKey();
        double frac = (timestampSeconds - t0) / (t1 - t0);

        Pose p0 = floor.getValue();
        Pose p1 = ceil.getValue();

        // linearly interpolate x,y; interpolate heading using smallest angle difference
        double ix = p0.getX() + frac * (p1.getX() - p0.getX());
        double iy = p0.getY() + frac * (p1.getY() - p0.getY());
        double dtheta = MathFunctions.getSmallestAngleDifference(p0.getHeading(), p1.getHeading());
        double iheading = p0.getHeading() + frac * dtheta;

        return new Pose(ix, iy, iheading);
    }

    /**
     * Samples the vision-compensated pose at timestampSeconds. Returns null if buffer empty.
     */
    public Pose sampleAt(double timestampSeconds) {
        if (m_odometryPoseBuffer.isEmpty()) {
            return null;
        }

        // clamp timestamp within odometry buffer range
        double oldest = m_odometryPoseBuffer.firstKey();
        double newest = m_odometryPoseBuffer.lastKey();
        timestampSeconds = Math.max(oldest, Math.min(newest, timestampSeconds));

        // If there are no applicable vision updates before this timestamp, just return odometry sample
        if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
            return getOdometrySample(timestampSeconds);
        }

        // get the latest vision update <= timestamp
        double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
        PoseTracker.VisionUpdate visionUpdate = m_visionUpdates.get(floorTimestamp);

        Pose odomSample = getOdometrySample(timestampSeconds);

        if (odomSample == null) return null;
        return visionUpdate.compensate(odomSample);
    }

    /** Removes vision updates that are too old to matter relative to the odometry buffer. */
    private void cleanUpVisionUpdates() {
        if (m_odometryPoseBuffer.isEmpty()) return;

        double oldestOdometryTimestamp = m_odometryPoseBuffer.firstKey();
        if (m_visionUpdates.isEmpty() || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
            return;
        }

        // newest needed vision update at or before oldest odometry timestamp
        double newestNeeded = m_visionUpdates.floorKey(oldestOdometryTimestamp);
        // remove strictly before newestNeeded
        m_visionUpdates.headMap(newestNeeded, false).clear();
    }

    /**
     * Add a vision measurement (pose) with a timestampSeconds (seconds epoch). This will:
     *  - reject if measurement too old for buffer,
     *  - compute the difference between vision pose and current vision-compensated pose at that time,
     *  - scale it by a per-axis Kalman-like gain,
     *  - store a VisionUpdate and update the current pose estimate.
     * Usage: when your vision system reports a pose, call:
     *   poseTracker.addVisionMeasurement(visionPose, System.nanoTime()/1e9);
     */
    public void addVisionMeasurement(Pose visionRobotPoseMeters, double timestampSeconds) {
        // Step 0: reject if too old (outside buffer window)
        if (m_odometryPoseBuffer.isEmpty()
            || m_odometryPoseBuffer.lastKey() - kBufferDuration > timestampSeconds) {
            return;
        }

        // Step 1: clean old entries
        cleanUpVisionUpdates();

        // Step 2: odometry pose at the moment the vision measurement was made
        Pose odomSample = getOdometrySample(timestampSeconds);
        if (odomSample == null) return;

        // Step 3: vision-compensated pose at measurement time
        Pose visionSample = sampleAt(timestampSeconds);
        if (visionSample == null) return;

        // Step 4: compute delta (twist) from visionSample to measured vision pose
        double dx = visionRobotPoseMeters.getX() - visionSample.getX();
        double dy = visionRobotPoseMeters.getY() - visionSample.getY();
        double raw = visionRobotPoseMeters.getHeading() - visionSample.getHeading();
        double dtheta = Math.copySign(
            MathFunctions.getSmallestAngleDifference(visionSample.getHeading(),
                visionRobotPoseMeters.getHeading()),
            raw);

        // Rotate delta into the local (visionSample) frame:
        double theta = visionSample.getHeading();
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        // R(-theta) * [dx_f; dy_f] gives local-frame translation
        double dx_local =  cos * dx + sin * dy;
        double dy_local = -sin * dx + cos * dy;

        // Step 5: scale by full 3x3 gain matrix (m_visionK is double[3][3], row-major)
        double[] localDelta = new double[]{dx_local, dy_local, dtheta};
        double[] k_times_twist = MathFunctions.mat3x3MulVec(m_visionK, localDelta);

        double scaled_dx_local = k_times_twist[0];
        double scaled_dy_local = k_times_twist[1];
        double scaled_dtheta   = k_times_twist[2];

        // Step 6: rotate scaled local corrections back to field frame:
        double corr_x_f = cos * scaled_dx_local - sin * scaled_dy_local;
        double corr_y_f = sin * scaled_dx_local + cos * scaled_dy_local;

        double currentUnwrappedHeading = visionSample.getHeading() + scaled_dtheta;
        double wrappedAngle =Math.atan2(
            Math.sin(currentUnwrappedHeading), Math.cos(currentUnwrappedHeading));

        // Apply correction to visionSample in field coords
        Pose correctedVisionPose = new Pose(
            visionSample.getX() + corr_x_f,
            visionSample.getY() + corr_y_f,
            wrappedAngle
        );

        // Step 7: record the vision update and remove later vision measurements
        PoseTracker.VisionUpdate visionUpdate = new PoseTracker.VisionUpdate(correctedVisionPose, odomSample);
        m_visionUpdates.put(timestampSeconds, visionUpdate);
        m_visionUpdates.tailMap(timestampSeconds, false).clear();

        // Step 8: update latest pose estimate by compensating current odometry pose
        Pose nowOdom = localizer.getPose();
        currentPose = visionUpdate.compensate(nowOdom);
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
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        currentPose = set.copy();
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
        if (Double.isNaN(getIMUHeadingEstimate()) || Double.isInfinite(getIMUHeadingEstimate())) {
            localizer.setPose(new Pose(getPose().getX(), getPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using offsets instead of Road
     * Runner's pose reset. This way, it's faster, but this can be wiped with the resetOffsets()
     * method.
     */
    public void resetHeadingToIMUWithOffsets() {
        if (Double.isNaN(getIMUHeadingEstimate()) || Double.isInfinite(getIMUHeadingEstimate())) {
            setCurrentPoseWithOffset(new Pose(getPose().getX(), getPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians.
     *
     * @return returns the normalized IMU heading.
     */
    public double getNormalizedIMUHeading() {
        if (Double.isNaN(getIMUHeadingEstimate()) || Double.isInfinite(getIMUHeadingEstimate())) {
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
    
    /**
     * Represents a vision update record. The record contains the vision-compensated pose estimate as
     * well as the corresponding odometry pose estimate.
     */
    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final Pose visionPose;

        // The pose estimated based solely on odometry.
        private final Pose odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(Pose visionPose, Pose odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
         * relative to this record's odometry pose to being relative to this record's vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public Pose compensate(Pose pose) {
            Pose delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }
}
