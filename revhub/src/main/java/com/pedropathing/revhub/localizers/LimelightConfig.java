package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;

public class LimelightConfig {
    public final ConfigVar<String> name = ConfigVar.required();
    public final ConfigVar<Integer> initialPipeline = ConfigVar.required();

    public final ConfigVar<CustomIMU> imu = ConfigVar.required();
    public LimelightConfig(Configuration<LimelightConfig> config) { config.configure(this); }
}
