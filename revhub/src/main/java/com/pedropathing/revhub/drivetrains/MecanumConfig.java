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

    /** Maximum speed that the robot can move forward/backward at, in inches per second. */
    public final ConfigVar<Double> maxForwardVelocity = ConfigVar.<Double>required().validate(Validator.positive());

    /** Maximum speed that the robot can move laterally, in inches per second. */
    public final ConfigVar<Double> maxStrafeVelocity = ConfigVar.<Double>required().validate(Validator.positive());

    /** Whether ZeroPowerBrake mode is enabled in manual mode. */
    public final ConfigVar<Boolean> manualBrakeMode = ConfigVar.of(false);

    /** Smallest power change that triggers a hardware write. */
    public final ConfigVar<Double> powerDeadband = ConfigVar.of(0.01);

    public MecanumConfig(Configuration<MecanumConfig> config) {
        config.configure(this);
    }
}