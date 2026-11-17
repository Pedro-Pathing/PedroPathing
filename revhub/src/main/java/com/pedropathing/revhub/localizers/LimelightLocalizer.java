package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.MotionState;
import com.pedropathing.localization.VisionLocalizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Localizer backed by a Limelight 3A running an AprilTag pipeline with MegaTag2 enabled.
 * <p>
 * The Limelight reports field positions in meters and degrees using the official FTC field
 * coordinate system with a centered origin. This localizer converts those measurements to Pedro
 * Pathing coordinates (inches and radians) so they can be used directly with the follower and
 * {@code FusionLocalizer}.
 * <p>
 * This class can be passed to the {@code FusionLocalizer} constructor, which will call
 * {@link #update()}, {@link #setPose(Pose)}, and {@link #reset()} as needed and fuse each new
 * measurement automatically.
 * <p>
 * MegaTag2 setup requirements: enable "Full 3D" in the AprilTag pipeline, configure the camera's
 * robot-space pose in the Limelight web interface, and upload a field map. This localizer calls
 * {@link Limelight3A#updateRobotOrientation(double)} every update and rejects measurements with no
 * tags or while the robot is rotating faster than the configured limit.
 */
public class LimelightLocalizer implements VisionLocalizer {
    private static final double FIELD_CENTER_INCHES = 72.0;

    /**
     * Pedro's documented FTC conversion is {@code pedroHeading = ftcHeading - PI / 2}. The
     * Limelight's FTC field frame uses the standard FTC heading, so the inverse offset converts
     * Pedro's heading to the yaw expected by {@link Limelight3A#updateRobotOrientation(double)}.
     */
    private static final double LIMELIGHT_YAW_OFFSET = Math.PI / 2;

    private final Limelight3A limelight;
    private final CustomIMU imu;
    private final Integer initialPipeline;
    private final double maxAngularVelocityDegrees;

    private MotionState motionState = MotionState.zero();
    private double headingOffset = 0.0;

    private LLResult lastResult;
    private double lastFrameTimestamp = -1;
    private Pose pendingMeasurement;
    private long lastMeasurementTimestamp = 0;

    public LimelightLocalizer(HardwareMap hardwareMap, LimelightConfig config) {
        limelight = hardwareMap.get(Limelight3A.class, config.name.get());

        imu = config.imu.get();
        imu.initialize(hardwareMap, config.imuName.get());

        initialPipeline = config.initialPipeline.get();
        maxAngularVelocityDegrees = config.maxAngularVelocityDegrees.get();

        limelight.pipelineSwitch(initialPipeline);
        limelight.start();
    }

    public void switchPipeline(Integer pipelineID) {
        limelight.pipelineSwitch(pipelineID);
    }

    /**
     * Sets the current pose estimate and records the offset between the IMU heading and the Pedro
     * world-frame heading. The offset is only needed for standalone use; fusion supplies the
     * heading directly.
     */
    @Override
    public void setPose(Pose pose) {
        headingOffset = Angle.normalizeSigned(pose.heading() - imu.getHeading());
        motionState = motionState.withPose(pose);
        pendingMeasurement = null;
        lastMeasurementTimestamp = 0;
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void update() {
        // Standalone use derives the field heading from the configured IMU.
        update(Angle.normalizeSigned(imu.getHeading() + headingOffset));
    }

    /**
     * Updates the Limelight using the supplied Pedro world-frame heading, which keeps MegaTag2's
     * orientation source synchronized with the fusion estimate.
     */
    @Override
    public void update(double robotHeading) {
        // MegaTag2 requires the robot's heading in the Limelight field frame every frame.
        double limelightYaw = Math.toDegrees(Angle.normalizeSigned(robotHeading + LIMELIGHT_YAW_OFFSET));
        limelight.updateRobotOrientation(limelightYaw);

        LLResult result = limelight.getLatestResult();
        if (result == null) return;

        // The SDK creates a new LLResult for every poll, even when the Limelight has not produced
        // a new pipeline frame. Use the Limelight frame timestamp to avoid fusing a frame twice;
        // fall back to object identity on older firmware that does not report "ts".
        double frameTimestamp = result.getTimestamp();
        if (frameTimestamp > 0) {
            if (frameTimestamp == lastFrameTimestamp) return;
            lastFrameTimestamp = frameTimestamp;
        } else if (result == lastResult) {
            return;
        }
        lastResult = result;

        // This result supersedes any measurement that has not been polled yet.
        pendingMeasurement = null;

        if (!result.isValid() || result.getBotposeTagCount() == 0) return;

        // Limelight recommends rejecting vision updates while the robot rotates quickly.
        double angularVelocity = Math.toDegrees(imu.getAngularVelocity());
        if (!Double.isNaN(angularVelocity) && Math.abs(angularVelocity) > maxAngularVelocityDegrees) {
            return;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return;

        // Limelight uses meters; Pedro Pathing and FusionLocalizer use inches.
        Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);

        // Limelight headings are degrees; Pedro Pathing uses radians.
        double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Convert the official FTC field frame (centered origin) to Pedro Pathing's frame
        // (corner origin, +x right, +y up).
        Pose pose = new Pose(
                position.y + FIELD_CENTER_INCHES,
                FIELD_CENTER_INCHES - position.x,
                heading - Math.PI / 2
        );

        // The Limelight is a pose sensor and does not provide velocity measurements.
        motionState = MotionState.ofVelocity(pose, Velocity.zero());

        // MegaTag2 derives its heading from the orientation we feed it, so only x and y are
        // independent measurements. Leave heading as NaN so the fusion localizer ignores it.
        pendingMeasurement = new Pose(pose.x(), pose.y(), Double.NaN);
        lastMeasurementTimestamp = calculateMeasurementTimestamp(result);
    }

    /**
     * Returns the next new measurement, or {@code null} if no new valid AprilTag result has
     * arrived. The returned pose contains only x and y; its heading is {@link Double#NaN}.
     */
    @Override
    public Pose pollMeasurement() {
        Pose measurement = pendingMeasurement;
        pendingMeasurement = null;
        return measurement;
    }

    @Override
    public long lastMeasurementTimestamp() {
        return lastMeasurementTimestamp;
    }

    /**
     * Converts the result's receive time into the {@link System#nanoTime()} time base and
     * compensates for the camera capture and pipeline latency.
     */
    private static long calculateMeasurementTimestamp(LLResult result) {
        long now = System.nanoTime();
        long controlHubTimestamp = result.getControlHubTimeStamp();
        long ageNanos = 0;
        if (controlHubTimestamp > 0) {
            long ageMillis = Math.max(0, System.currentTimeMillis() - controlHubTimestamp);
            ageNanos = ageMillis * 1_000_000L;
        }

        double latencyMillis = result.getCaptureLatency() + result.getTargetingLatency();
        if (Double.isNaN(latencyMillis) || Double.isInfinite(latencyMillis) || latencyMillis < 0) {
            latencyMillis = 0;
        }

        return now - ageNanos - (long) (latencyMillis * 1_000_000.0);
    }

    @Override
    public void reset() {
        imu.resetYaw();
        headingOffset = 0.0;
        motionState = MotionState.zero();
        pendingMeasurement = null;
        lastMeasurementTimestamp = 0;
    }
}
