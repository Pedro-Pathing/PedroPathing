package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelLocalizer implements Localizer {
    private final CustomIMU imu;
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

    public TwoWheelLocalizer(HardwareMap map, TwoWheelConfig config) {
        this.forwardTicksToInches = config.forwardTicksToInches.get();
        this.strafeTicksToInches = config.strafeTicksToInches.get();
        this.xPodOffset = config.xPodOffset.get();
        this.yPodOffset = config.yPodOffset.get();

        this.imu = config.imu.get();
        this.imu.initialize(map, config.imuName.get(), config.imuOrientation.get());

        this.xPodEncoder = new Encoder(map.get(DcMotorEx.class, config.xPodName.get()));
        this.yPodEncoder = new Encoder(map.get(DcMotorEx.class, config.yPodName.get()));

        this.xPodEncoder.setDirection(config.xPodDirection.get());
        this.yPodEncoder.setDirection(config.yPodDirection.get());

        this.timer = new Timer();
        this.previousIMUOrientation = Angle.normalize(imu.getHeading());

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

        double currentIMUOrientation = Angle.normalize(imu.getHeading());
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
        imu.resetYaw();
        xPodEncoder.reset();
        yPodEncoder.reset();
        pose = Pose.zero();
        previousIMUOrientation = Angle.normalize(imu.getHeading());
    }
}
