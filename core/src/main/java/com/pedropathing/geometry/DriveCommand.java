package com.pedropathing.geometry;

public class DriveCommand {
    private static final DriveCommand ZERO = new DriveCommand(0, 0, 0);
    public final double forward;
    public final double strafe;
    public final double turn;

    public DriveCommand(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    public static DriveCommand zero() {
        return ZERO;
    }
}
