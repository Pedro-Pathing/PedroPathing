package com.pedropathing.revhub.localizers;

import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimelightLocalizer implements Localizer {
    private final Limelight3A limelight;
    private final CustomIMU imu;
    private final Integer initialPipeline;
    public LimelightLocalizer(HardwareMap hardwareMap, LimelightConfig config) {
        limelight = hardwareMap.get(Limelight3A.class, config.name.get());

        imu = config.imu.get();

        initialPipeline = config.initialPipeline.get();

        limelight.pipelineSwitch(initialPipeline);
    }
    public void switchPipeline(Integer pipelineID) {
        limelight.pipelineSwitch(pipelineID);
    }
}
