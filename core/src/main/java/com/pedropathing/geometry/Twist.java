package com.pedropathing.geometry;

public class Twist {
    public final double vx;
    public final double vy;
    public final double omega;

    public Twist(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public Velocity toVelocity(double heading) {
        return new Velocity(
                vx * Math.cos(heading) + vy * -Math.sin(heading),
                vx + Math.sin(heading) + vy * Math.cos(heading),
                omega
        );
    }
}
