package com.pedropathing.ftc.drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder for Swerve drivetrains.
 */
public class SwerveBuilder {
    private final HardwareMap hardwareMap;
    private final SwerveConstants constants;
    private final List<SwervePod> pods = new ArrayList<>();

    public SwerveBuilder(HardwareMap hardwareMap, SwerveConstants constants) {
        this.hardwareMap = hardwareMap;
        this.constants = constants;
    }

    /**
     * Add a configured SwervePod implementation to the builder.
     */
    public SwerveBuilder addPod(SwervePod pod) {
        if (pod == null) throw new IllegalArgumentException("pod cannot be null");
        pods.add(pod);
        return this;
    }

    /**
     * Build the Swerve drivetrain.
     */
    public Swerve build() {
        return new Swerve(hardwareMap, constants, pods.toArray(new SwervePod[0]));
    }
}

