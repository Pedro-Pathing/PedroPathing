package com.pedropathing.revhub.drivetrains;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.config.Validator.nonnegative;

public class CoaxialPodConfig {
    public final ConfigVar<String> motorName = ConfigVar.required();
    public final ConfigVar<String> servoName = ConfigVar.required();
    public final ConfigVar<String> servoEncoderName = ConfigVar.required();

    /** PIDF gains for turn servo control. */
    public final ConfigVar<Controller> turnController = ConfigVar.of(Controller.zero);

    public final ConfigVar<DcMotorSimple.Direction> driveDirection = ConfigVar.required();
    public final ConfigVar<CRServo.Direction> servoDirection = ConfigVar.required();

    /** Raw encoder angle (radians) when the wheel is facing forward. */
    public final ConfigVar<Double> angleOffsetRad = ConfigVar.of(0.0);

    /** Pod position offset from robot center. */
    public final ConfigVar<Vector2D> podOffset = ConfigVar.required();

    public final ConfigVar<Double> analogMinVoltage = ConfigVar.of(0.0, nonnegative());
    public final ConfigVar<Double> analogMaxVoltage = ConfigVar.of(3.3, nonnegative());

    /** True if encoder increases CCW (top-down). */
    public final ConfigVar<Boolean> encoderReversed = ConfigVar.of(false);

    /** Smallest power change that triggers a hardware write. */
    public final ConfigVar<Double> motorCachingThreshold = ConfigVar.of(0.05, nonnegative());

    /** Smallest position change that triggers a hardware write. */
    public final ConfigVar<Double> servoCachingThreshold = ConfigVar.of(0.05, nonnegative());

    public CoaxialPodConfig(Configuration<CoaxialPodConfig> config) {
        config.configure(this);
    }
}