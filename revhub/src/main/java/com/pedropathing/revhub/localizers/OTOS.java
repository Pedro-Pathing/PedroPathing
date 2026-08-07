package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OTOS implements Localizer {
    private final SparkFunOTOS otos;

    private MotionState motionState;

    public OTOS(HardwareMap hardwareMap, OTOSConfig config) {
        otos = hardwareMap.get(SparkFunOTOS.class, config.name.get());

        otos.setLinearUnit(config.linearUnit.get());
        otos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offsetPose = new SparkFunOTOS.Pose2D(
                config.offset.get().x(),
                config.offset.get().y(),
                config.offset.get().heading() + Math.PI / 2
        );
        otos.setOffset(offsetPose);
        otos.setLinearScalar(config.linearScalar.get());
        otos.setAngularScalar(config.angularScalar.get());

        otos.calibrateImu();
        otos.resetTracking();

        update();
    }

    public void setPose(Pose pose) {
        otos.setPosition(
                new SparkFunOTOS.Pose2D(
                        pose.x(),
                        pose.y(),
                        pose.heading() + Math.PI / 2
                )
        );

        if (motionState != null) {
            motionState = motionState.withPose(pose);
        } else {
            motionState = MotionState.ofVelocity(pose, Velocity.zero());
        }
    }

    @Override
    public void update() {
        SparkFunOTOS.Pose2D pose2D = otos.getPosition();
        SparkFunOTOS.Pose2D velocity2D = otos.getVelocity();

        Pose pose = new Pose(
                pose2D.x,
                pose2D.y,
                pose2D.h - Math.PI / 2
        );

        Velocity velocity = new Velocity(
                velocity2D.x,
                velocity2D.y,
                velocity2D.h
        );

        motionState = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    public void reset() {
        otos.resetTracking();
    }
}