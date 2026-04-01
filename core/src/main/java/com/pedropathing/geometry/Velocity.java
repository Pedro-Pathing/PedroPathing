package com.pedropathing.geometry;

public class Velocity {
    public final double vx;
    public final double vy;
    public final double omega;

    public Velocity(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public Twist toTwist(double heading) {
        return new Twist(
                vx * Math.cos(heading) + vy * Math.sin(heading),
                vx * -Math.sin(heading) + vy * Math.cos(heading),
                omega
        );
    }
}
