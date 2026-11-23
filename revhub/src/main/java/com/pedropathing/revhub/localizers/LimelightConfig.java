package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;

public class LimelightConfig {
    public final ConfigVar<String> name = ConfigVar.required();
    public final ConfigVar<Integer> initialPipeline = ConfigVar.required();

    public final ConfigVar<CustomIMU> imu = ConfigVar.required();
    public final ConfigVar<String> imuName = ConfigVar.required();

    /**
     * Maximum absolute rotation rate, in degrees per second, at which MegaTag2 measurements are
     * accepted. Limelight recommends rejecting poses while turning quickly. Set to
     * {@link Double#POSITIVE_INFINITY} to disable the check.
     */
    public final ConfigVar<Double> maxAngularVelocityDegrees = ConfigVar.of(360.0);

    public LimelightConfig(Configuration<LimelightConfig> config) { config.configure(this); }
}
