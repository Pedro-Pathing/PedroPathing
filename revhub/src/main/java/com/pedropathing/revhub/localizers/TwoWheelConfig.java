package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class TwoWheelConfig {
    public final ConfigVar<String> xPodName = ConfigVar.required();
    public final ConfigVar<String> yPodName = ConfigVar.required();
    public final ConfigVar<String> imuName = ConfigVar.required();

    public final ConfigVar<Double> xPodOffset = ConfigVar.required();
    public final ConfigVar<Double> yPodOffset = ConfigVar.required();

    public final ConfigVar<Double> forwardTicksToInches = ConfigVar.required();
    public final ConfigVar<Double> strafeTicksToInches = ConfigVar.required();

    public final ConfigVar<Double> xPodDirection = ConfigVar.required();
    public final ConfigVar<Double> yPodDirection = ConfigVar.required();

    public final ConfigVar<CustomIMU> imu = ConfigVar.required();

    public TwoWheelConfig(Configuration<TwoWheelConfig> config) {
        config.configure(this);
    }
}
