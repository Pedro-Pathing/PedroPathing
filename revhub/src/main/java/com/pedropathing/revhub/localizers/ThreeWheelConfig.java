package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;

public class ThreeWheelConfig {
    public final ConfigVar<String> leftEncoderName = ConfigVar.required();
    public final ConfigVar<String> rightEncoderName = ConfigVar.required();
    public final ConfigVar<String> strafeEncoderName = ConfigVar.required();

    public final ConfigVar<Double> leftPodY = ConfigVar.required();
    public final ConfigVar<Double> rightPodY = ConfigVar.required();
    public final ConfigVar<Double> strafePodX = ConfigVar.required();

    public final ConfigVar<Double> forwardTicksToInches = ConfigVar.required();
    public final ConfigVar<Double> strafeTicksToInches = ConfigVar.required();
    public final ConfigVar<Double> turnTicksToRadians = ConfigVar.required();

    public final ConfigVar<Double> leftEncoderDirection = ConfigVar.required();
    public final ConfigVar<Double> rightEncoderDirection = ConfigVar.required();
    public final ConfigVar<Double> strafeEncoderDirection = ConfigVar.required();

    public ThreeWheelConfig(Configuration<ThreeWheelConfig> config) {
        config.configure(this);
    }
}