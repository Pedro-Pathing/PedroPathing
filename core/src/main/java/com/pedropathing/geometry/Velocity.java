package com.pedropathing.geometry;

public class Velocity {
    public final double vx;
    public final double vy;
    public final double omega;
    private static final Velocity ZERO = new Velocity(0, 0, 0);

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

    public Velocity add(Velocity other) {
        return new Velocity(vx + other.vx, vy + other.vy, omega + other.omega);
    }

    public Velocity scale(double time) {
        return new Velocity(vx * time, vy * time, omega * time);
    }

    public Vector2D toLinear() {
        return new Vector2D(vx, vy);
    }

    public static Velocity zero() {
        return ZERO;
    }
}
