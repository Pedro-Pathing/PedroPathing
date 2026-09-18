package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.HeadingSource;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TwoWheelLocalizer implements Localizer {
    private final CustomIMU imu;
    private final HeadingSource headingSource;
    private final boolean resetImuOnRequest;
    private final Encoder xPodEncoder;
    private final Encoder yPodEncoder;

    private final double forwardTicksToInches;
    private final double strafeTicksToInches;
    private final double xPodOffset;
    private final double yPodOffset;

    private MotionState motionState;
    private Pose pose = Pose.zero();

    private final Timer timer;
    private double previousIMUOrientation;

    /**
     * Creates a hardware-backed localizer from a HardwareMap. Encoders and the IMU are
     * constructed from the configured names and then delegated to the source-injection path.
     */
    public TwoWheelLocalizer(HardwareMap map, TwoWheelConfig config) {
        this(config,
                initializedHeading(map, config),
                new Encoder(map.get(DcMotorEx.class, config.xPodName.get())),
                new Encoder(map.get(DcMotorEx.class, config.yPodName.get())),
                true);
    }

    /**
     * Creates a localizer from injected encoder and heading sources. Encoder values are raw
     * hardware-sign ticks; Pedro applies the directions in {@code config}. Heading is in
     * radians. Reset rebases encoder software zeros and does not physically reset sensors.
     */
    public TwoWheelLocalizer(TwoWheelConfig config, IntSupplier xPodPosition,
                             IntSupplier yPodPosition, DoubleSupplier headingRadians) {
        this(config, headingRadians::getAsDouble, Encoder.from(xPodPosition), Encoder.from(yPodPosition), false);
    }

    private static HeadingSource initializedHeading(HardwareMap map, TwoWheelConfig config) {
        CustomIMU imu = config.imu.get();
        imu.initialize(map, config.imuName.get());
        return imu::getHeading;
    }

    private TwoWheelLocalizer(TwoWheelConfig config, HeadingSource headingSource,
                              Encoder xPodEncoder, Encoder yPodEncoder, boolean resetImuOnRequest) {
        this.forwardTicksToInches = config.forwardTicksToInches.get();
        this.strafeTicksToInches = config.strafeTicksToInches.get();
        this.xPodOffset = config.xPodOffset.get();
        this.yPodOffset = config.yPodOffset.get();

        this.imu = resetImuOnRequest ? config.imu.get() : null;
        this.headingSource = headingSource;
        this.resetImuOnRequest = resetImuOnRequest;

        this.xPodEncoder = xPodEncoder;
        this.yPodEncoder = yPodEncoder;

        this.xPodEncoder.setDirection(config.xPodDirection.get());
        this.yPodEncoder.setDirection(config.yPodDirection.get());

        this.timer = new Timer();
        this.previousIMUOrientation = Angle.normalize(headingSource.headingRadians());

        this.motionState = MotionState.ofVelocity(pose, Velocity.zero());
        update();
    }

    public void setPose(Pose setPose) {
        this.pose = setPose;
        xPodEncoder.reset();
        yPodEncoder.reset();

        if (motionState != null) {
            motionState = motionState.withPose(setPose);
        } else {
            motionState = MotionState.ofVelocity(setPose, Velocity.zero());
        }
    }

    @Override
    public void update() {
        long deltaTimeNano = timer.nanoseconds();
        timer.reset();

        xPodEncoder.update();
        yPodEncoder.update();

        double currentIMUOrientation = Angle.normalize(headingSource.headingRadians());
        double deltaRadians = Angle.turnDirection(previousIMUOrientation, currentIMUOrientation)
                * Angle.smallestDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;

        double deltaX = forwardTicksToInches * xPodEncoder.getDeltaPosition() - xPodOffset * deltaRadians;
        double deltaY = strafeTicksToInches * yPodEncoder.getDeltaPosition() - yPodOffset * deltaRadians;

        Matrix prevRotationMatrix = Matrix.rotationTransform(pose.heading());

        Matrix transformation;
        if (Math.abs(deltaRadians) < 0.001) {
            double term = 1.0 - (Math.pow(deltaRadians, 2) / 6.0);
            double halfDelta = deltaRadians / 2.0;
            transformation = new Matrix(new double[][]{
                    {term, -halfDelta, 0.0},
                    {halfDelta, term, 0.0},
                    {0.0, 0.0, 1.0}
            });
        } else {
            double sinD = Math.sin(deltaRadians);
            double cosD = Math.cos(deltaRadians);
            transformation = new Matrix(new double[][]{
                    {sinD / deltaRadians, (cosD - 1.0) / deltaRadians, 0.0},
                    {(1.0 - cosD) / deltaRadians, sinD / deltaRadians, 0.0},
                    {0.0, 0.0, 1.0}
            });
        }

        Matrix robotDeltas = new Matrix(new double[][]{
                {deltaX},
                {deltaY},
                {deltaRadians}
        });

        Matrix globalDeltas = prevRotationMatrix.times(transformation).times(robotDeltas);

        pose = pose.plus(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));

        double dtSeconds = deltaTimeNano / 1e9;
        Velocity currentVelocity = new Velocity(
                globalDeltas.get(0, 0) / dtSeconds,
                globalDeltas.get(1, 0) / dtSeconds,
                globalDeltas.get(2, 0) / dtSeconds
        );

        motionState = MotionState.ofVelocity(pose, currentVelocity);
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    public void reset() {
        if (resetImuOnRequest) {
            imu.resetYaw();
        }
        xPodEncoder.reset();
        yPodEncoder.reset();
        pose = Pose.zero();
        previousIMUOrientation = Angle.normalize(headingSource.headingRadians());
    }
}
