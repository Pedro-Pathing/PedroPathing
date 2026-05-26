package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
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
    private MotionState motionState;

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
        odometry.setPosition(
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
                odometry.getPosX(globalDistanceUnit),
                odometry.getPosY(globalDistanceUnit),
                odometry.getHeading(AngleUnit.RADIANS)
        );

        Velocity velocity = new Velocity(
                odometry.getVelX(globalDistanceUnit),
                odometry.getVelY(globalDistanceUnit),
                odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );

        Twist twist = velocity.toTwist(pose.heading());

        motionState = new MotionState(pose, velocity, twist);
    }

    @Override
    public MotionState motionState() {
        return motionState;
    }

    public void reset() {
        odometry.resetPosAndIMU();
    }
}