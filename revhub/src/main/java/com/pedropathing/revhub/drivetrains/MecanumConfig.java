package com.pedropathing.revhub.drivetrains;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumConfig {
    public final ConfigVar<String> leftFrontName = ConfigVar.required();
    public final ConfigVar<String> leftRearName = ConfigVar.required();
    public final ConfigVar<String> rightFrontName = ConfigVar.required();
    public final ConfigVar<String> rightRearName = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> leftFrontDirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> leftRearDirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> rightFrontDirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> rightRearDirection = ConfigVar.required();
    public final ConfigVar<Boolean> manualBrakeMode = ConfigVar.of(false);
    public final ConfigVar<Double> powerThreshold = ConfigVar.of(0.01);

    public MecanumConfig(Configuration<MecanumConfig> config) {
        config.configure(this);
    }
}