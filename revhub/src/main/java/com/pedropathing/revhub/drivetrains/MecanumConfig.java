package com.pedropathing.revhub.drivetrains;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.pedropathing.config.Validator;
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

    public final ConfigVar<Double> maxForwardVelocity = ConfigVar.<Double>required().validate(Validator.positive());
    public final ConfigVar<Double> maxStrafeVelocity = ConfigVar.<Double>required().validate(Validator.positive());
    public final ConfigVar<Boolean> manualBrakeMode = ConfigVar.of(false);

    public MecanumConfig(Configuration<MecanumConfig> config) {
        config.configure(this);
    }
}