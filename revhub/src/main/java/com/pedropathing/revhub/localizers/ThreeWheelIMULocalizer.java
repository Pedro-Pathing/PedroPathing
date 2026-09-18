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

public class ThreeWheelIMULocalizer implements Localizer {
    private final CustomIMU imu;
    private final HeadingSource headingSource;
    private final boolean resetImuOnRequest;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder strafeEncoder;

    private final double leftPodY;
    private final double rightPodY;
    private final double strafePodX;

    private final double forwardTicksToInches;
    private final double strafeTicksToInches;
    private final double turnTicksToRadians;

    private MotionState motionState;
    private Pose pose = Pose.zero();
    private double previousIMUOrientation;
    private double totalHeading = 0;

    private final Timer timer;
    public static boolean useIMU = true;

    /**
     * Creates a hardware-backed localizer from a HardwareMap. Encoders and the IMU are
     * constructed from the configured names and then delegated to the source-injection path.
     */
    public ThreeWheelIMULocalizer(HardwareMap map, ThreeWheelIMUConfig config) {
        this(config,
                initializedHeading(map, config),
                new Encoder(map.get(DcMotorEx.class, config.leftEncoderName.get())),
                new Encoder(map.get(DcMotorEx.class, config.rightEncoderName.get())),
                new Encoder(map.get(DcMotorEx.class, config.strafeEncoderName.get())),
                true);
    }

    /**
     * Creates a localizer from injected encoder and heading sources. Encoder values are raw
     * hardware-sign ticks; Pedro applies the directions in {@code config}. Heading is in
     * radians. Reset rebases encoder software zeros and does not physically reset sensors.
     */
    public ThreeWheelIMULocalizer(ThreeWheelIMUConfig config, IntSupplier leftPosition,
                                  IntSupplier rightPosition, IntSupplier strafePosition,
                                  DoubleSupplier headingRadians) {
        this(config, headingRadians::getAsDouble,
                Encoder.from(leftPosition), Encoder.from(rightPosition), Encoder.from(strafePosition),
                false);
    }

    private static HeadingSource initializedHeading(HardwareMap map, ThreeWheelIMUConfig config) {
        CustomIMU imu = config.imu.get();
        imu.initialize(map, config.imuName.get());
        return imu::getHeading;
    }

    private ThreeWheelIMULocalizer(ThreeWheelIMUConfig config, HeadingSource headingSource,
                                   Encoder leftEncoder, Encoder rightEncoder, Encoder strafeEncoder,
                                   boolean resetImuOnRequest) {
        this.forwardTicksToInches = config.forwardTicksToInches.get();
        this.strafeTicksToInches = config.strafeTicksToInches.get();
        this.turnTicksToRadians = config.turnTicksToRadians.get();

        this.leftPodY = config.leftPodY.get();
        this.rightPodY = config.rightPodY.get();
        this.strafePodX = config.strafePodX.get();

        this.imu = resetImuOnRequest ? config.imu.get() : null;
        this.headingSource = headingSource;
        this.resetImuOnRequest = resetImuOnRequest;

        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.strafeEncoder = strafeEncoder;

        this.leftEncoder.setDirection(config.leftEncoderDirection.get());
        this.rightEncoder.setDirection(config.rightEncoderDirection.get());
        this.strafeEncoder.setDirection(config.strafeEncoderDirection.get());

        this.timer = new Timer();
        this.previousIMUOrientation = Angle.normalize(headingSource.headingRadians());

        this.motionState = MotionState.ofVelocity(pose, Velocity.zero());
        update();
    }

    public void setPose(Pose setPose) {
        this.pose = setPose;
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();

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

        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();

        double currentIMUOrientation = Angle.normalize(headingSource.headingRadians());
        double imuDeltaRadians = Angle.turnDirection(previousIMUOrientation, currentIMUOrientation)
                * Angle.smallestDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;

        double deltaLeft = leftEncoder.getDeltaPosition();
        double deltaRight = rightEncoder.getDeltaPosition();
        double deltaStrafe = strafeEncoder.getDeltaPosition();

        double podYDiff = leftPodY - rightPodY;

        double deltaX = forwardTicksToInches * (deltaRight * leftPodY - deltaLeft * rightPodY) / podYDiff;
        double deltaY = strafeTicksToInches * (deltaStrafe - strafePodX * ((deltaRight - deltaLeft) / podYDiff));

        double deltaRadians;
        if (Angle.smallestDifference(0, imuDeltaRadians) > 0.00005 && useIMU) {
            deltaRadians = imuDeltaRadians;
        } else {
            deltaRadians = turnTicksToRadians * (deltaRight - deltaLeft) / podYDiff;
        }

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
        totalHeading += globalDeltas.get(2, 0);

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

    public double getTotalHeading() {
        return totalHeading;
    }

    public void reset() {
        if (resetImuOnRequest) {
            imu.resetYaw();
        }
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();
        pose = Pose.zero();
        previousIMUOrientation = Angle.normalize(headingSource.headingRadians());
        totalHeading = 0;
    }
}
