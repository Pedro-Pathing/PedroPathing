package com.pedropathing.geometry;

public class DrivePowers {
    private static final DrivePowers ZERO = new DrivePowers(0, 0, 0);
    public final double forward;
    public final double strafe;
    public final double turn;

    public DrivePowers(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    public static DrivePowers zero() {
        return ZERO;
    }
}
