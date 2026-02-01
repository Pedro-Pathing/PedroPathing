package com.pedropathing.ftc.drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder for Swerve drivetrains.
 * @author Kabir Goyal
 */
public class SwerveBuilder {
    private final HardwareMap hardwareMap;
    private final SwerveConstants constants;
    private final List<SwervePod> pods = new ArrayList<>();

    /**
     * @param constants Swerve Constants for your bot
     */
    public SwerveBuilder(HardwareMap hardwareMap, SwerveConstants constants) {
        this.hardwareMap = hardwareMap;
        this.constants = constants;
    }

    /**
     * Add a configured SwervePod implementation to the builder.
     * Pods should be fully configured before building the drivetrain.
     *
     * @param pod configured swerve pod
     * @return this builder
     */
    public SwerveBuilder addPod(SwervePod pod) {
        if (pod == null) throw new IllegalArgumentException("pod cannot be null");
        pods.add(pod);
        return this;
    }

    /**
     * Build the Swerve drivetrain with the added pods.
     *
     * @return constructed swerve drivetrain
     */
    public Swerve build() {
        return new Swerve(hardwareMap, constants, pods.toArray(new SwervePod[0]));
    }
}
