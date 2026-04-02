package com.pedropathing.geometry;

public class Vector {
    public final double x;
    public final double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector normalized() {
        return new Vector(x / magnitude(), y / magnitude());
    }

    public Vector plus(Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector minus(Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public Vector times(double scalar) {
        return new Vector(x * scalar, y * scalar);
    }

    public Vector div(double scalar) {
        return new Vector(x / scalar, y / scalar);
    }

    public Vector rotate(double angle) {
        return new Vector(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double dot(Vector other) {
        return x * other.x + y * other.y;
    }

    public Vector cross(Vector other) {
        return new Vector(x * other.y - y * other.x, x * other.x + y * other.y);
    }

    public Vector projectOnto(Vector other) {
        return other.times(dot(other) / other.dot(other));
    }

    public double theta() {
        return Math.atan2(y, x);
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    public static Vector polar(double radius, double angle) {
        return new Vector(radius * Math.cos(angle), radius * Math.sin(angle));
    }

    public static Vector cartesian(double x, double y) {
        return new Vector(x, y);
    }

    private static final Vector ZERO = new Vector(0, 0);
    private static final Vector I_HAT = new Vector(1, 0);
    private static final Vector J_HAT = new Vector(0, 1);

    public static Vector zero() {
        return ZERO;
    }

    public static Vector iHat() {
        return I_HAT;
    }

    public static Vector jHat() {
        return J_HAT;
    }
}
