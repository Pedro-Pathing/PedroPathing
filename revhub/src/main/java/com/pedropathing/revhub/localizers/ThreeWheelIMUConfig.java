package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class ThreeWheelIMUConfig {
    public final ConfigVar<String> leftEncoderName = ConfigVar.required();
    public final ConfigVar<String> rightEncoderName = ConfigVar.required();
    public final ConfigVar<String> strafeEncoderName = ConfigVar.required();
    public final ConfigVar<String> imuName = ConfigVar.required();

    public final ConfigVar<Double> leftPodY = ConfigVar.required();
    public final ConfigVar<Double> rightPodY = ConfigVar.required();
    public final ConfigVar<Double> strafePodX = ConfigVar.required();

    public final ConfigVar<Double> forwardTicksToInches = ConfigVar.required();
    public final ConfigVar<Double> strafeTicksToInches = ConfigVar.required();
    public final ConfigVar<Double> turnTicksToRadians = ConfigVar.required();

    public final ConfigVar<Double> leftEncoderDirection = ConfigVar.required();
    public final ConfigVar<Double> rightEncoderDirection = ConfigVar.required();
    public final ConfigVar<Double> strafeEncoderDirection = ConfigVar.required();

    public final ConfigVar<RevHubOrientationOnRobot> imuOrientation = ConfigVar.required();
    public final ConfigVar<CustomIMU> imu = ConfigVar.of(new RevHubIMU());

    public ThreeWheelIMUConfig(Configuration<ThreeWheelIMUConfig> config) {
        config.configure(this);
    }
}