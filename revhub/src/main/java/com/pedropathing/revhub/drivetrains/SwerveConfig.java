package com.pedropathing.revhub.drivetrains;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;

/**
 * Constants for swerve drive configuration.
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class SwerveConfig {
    public final ConfigVar<Boolean> manualBrakeMode = ConfigVar.of(true);
    public final ConfigVar<Boolean> voltageCompensation = ConfigVar.required();
    public final ConfigVar<Double> nominalVoltage = ConfigVar.of(12.0);
    public final ConfigVar<Double> staticFrictionCoefficient = ConfigVar.of(0.1);
    public final ConfigVar<Double> epsilon = ConfigVar.of(0.05);

    public enum ZeroPowerBehavior {
        X_LOCK,
        IGNORE_ANGLE_CHANGES
    }

    public final ConfigVar<ZeroPowerBehavior> zeroPowerBehavior = ConfigVar.required();
    public SwerveConfig(Configuration<SwerveConfig> config) {
        config.configure(this);
    }
}