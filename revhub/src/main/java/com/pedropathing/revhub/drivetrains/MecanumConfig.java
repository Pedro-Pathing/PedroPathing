package com.pedropathing.revhub.drivetrains;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.config.Validator.nonnegative;

public class MecanumConfig {
    public final ConfigVar<String> frontLeftName = ConfigVar.required();
    public final ConfigVar<String> backLeftName = ConfigVar.required();
    public final ConfigVar<String> frontRightName = ConfigVar.required();
    public final ConfigVar<String> backRightName = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> frontLeftDirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> backLeftDirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> frontRightDirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> backRightDirection = ConfigVar.required();

    /** Whether ZeroPowerBrake mode is enabled in manual mode. */
    public final ConfigVar<Boolean> manualBrakeMode = ConfigVar.of(true);

    /** Smallest power change that triggers a hardware write. */
    public final ConfigVar<Double> powerThreshold = ConfigVar.of(0.01, nonnegative());

    public MecanumConfig(Configuration<MecanumConfig> config) {
        config.configure(this);
    }
}