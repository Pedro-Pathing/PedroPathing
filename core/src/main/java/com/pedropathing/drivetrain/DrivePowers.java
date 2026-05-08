package com.pedropathing.drivetrain;

import lombok.Value;

@Value
public class DrivePowers {
    private static final DrivePowers ZERO = new DrivePowers(0, 0, 0);
    double forward;
    double strafe;
    double turn;

    public static DrivePowers zero() {
        return ZERO;
    }
}
