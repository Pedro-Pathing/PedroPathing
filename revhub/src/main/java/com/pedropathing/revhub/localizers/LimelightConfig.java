package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;

import java.util.List;

public class LimelightConfig {
    public final ConfigVar<String> name = ConfigVar.required();
    public final ConfigVar<List<Integer>> pipelineIds = ConfigVar.required();
    public LimelightConfig(Configuration<LimelightConfig> config) { config.configure(this); }
}
