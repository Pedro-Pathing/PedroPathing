package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Pinpoint implements Localizer {
    private final GoBildaPinpointDriver odometry;
    private final DistanceUnit globalDistanceUnit;

    private Pose pose;
    private Velocity velocity;
    private Twist twist;

    public Pinpoint(HardwareMap hardwareMap, PinpointConfig config) {
        this.globalDistanceUnit = config.globalDistanceUnit.get();

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());

        odometry.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());

        if (config.ticksPerUnit.get().isPresent()) {
            odometry.setEncoderResolution(config.ticksPerUnit.get().get(), config.encoderResolutionUnit.get());
        } else {
            odometry.setEncoderResolution(config.podType.get());
        }

        odometry.setEncoderDirections(
                config.xPodDirection.get(),
                config.yPodDirection.get()
        );

        update();
    }

    public void setPose(Pose pose) {
        odometry.setPosition( // TODO: need to impl conversion from sdk to pedro
                new Pose2D(
                        globalDistanceUnit,
                        pose.x(),
                        pose.y(),
                        AngleUnit.RADIANS,
                        pose.heading()
                )
        );

        this.pose = pose;
    }

    @Override
    public void update() {
        odometry.update();

        pose = new Pose(
                odometry.getPosX(globalDistanceUnit), // TODO: need to impl conversion from sdk to pedro
                odometry.getPosY(globalDistanceUnit),
                odometry.getHeading(AngleUnit.RADIANS)
        );

        velocity = new Velocity(
                odometry.getVelX(globalDistanceUnit), // TODO: need to impl conversion from sdk to pedro
                odometry.getVelY(globalDistanceUnit),
                odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );

        twist = velocity.toTwist(pose.heading());
    }

    @Override
    public Twist getTwist() {
        return twist;
    }

    @Override
    public Velocity getVelocity() {
        return velocity;
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    public void reset() {
        odometry.recalibrateIMU();
    }
}