package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OctoQuad implements Localizer {
    private final com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerDataBlock localizer = new com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerDataBlock();
    private final com.qualcomm.hardware.digitalchickenlabs.OctoQuad octoQuad;
    private final DistanceUnit globalDistanceUnit;

    private MotionState motionState;

    public OctoQuad(HardwareMap hardwareMap, OctoQuadConfig config) {
        octoQuad = hardwareMap.get(com.qualcomm.hardware.digitalchickenlabs.OctoQuad.class, config.name.get());

        globalDistanceUnit = config.globalDistanceUnit.get();

        octoQuad.setSingleEncoderDirection(config.xPodPort.get(), config.xPodDirection.get());
        octoQuad.setSingleEncoderDirection(config.yPodPort.get(), config.yPodDirection.get());

        float ticksPerMM = (float) DistanceUnit.MM.fromUnit(config.encoderResolutionUnit.get(), config.ticksPerUnit.get());
        octoQuad.setAllLocalizerParameters(
                config.xPodPort.get(),
                config.yPodPort.get(),
                ticksPerMM,
                ticksPerMM,
                (float) -config.offsetUnits.get().toMm(config.xPodOffset.get()),
                (float) -config.offsetUnits.get().toMm(config.yPodOffset.get()),
                config.headingScalar.get().floatValue(),
                config.localizerVelocityIntervalMS.get()
        );
        octoQuad.setI2cRecoveryMode(config.i2cRecoveryMode.get());

        reset();

        while (octoQuad.getLocalizerStatus() != com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerStatus.RUNNING) {}

        update();
    }

    @Override
    public void update() {
        octoQuad.readLocalizerData(localizer);

        if (!localizer.isDataValid()) {
            return;
        }

        Pose pose = new Pose(
                globalDistanceUnit.fromMm(localizer.posX_mm),
                globalDistanceUnit.fromMm(localizer.posY_mm),
                localizer.heading_rad
        );

        Velocity velocity = new Velocity(
                globalDistanceUnit.fromMm(localizer.velX_mmS),
                globalDistanceUnit.fromMm(localizer.velY_mmS),
                localizer.velHeading_radS
        );

        motionState = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public void setPose(Pose pose) {
        octoQuad.setLocalizerPose((int) globalDistanceUnit.toMm(pose.x()), (int) globalDistanceUnit.toMm(pose.y()), (float) pose.heading());

        if (motionState != null) {
            motionState = motionState.withPose(pose);
        } else {
            motionState = MotionState.ofVelocity(pose, Velocity.zero());
        }
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void reset() {
        octoQuad.resetLocalizerAndCalibrateIMU();
    }
}
