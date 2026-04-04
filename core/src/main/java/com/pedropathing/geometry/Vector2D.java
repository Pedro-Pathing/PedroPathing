package com.pedropathing.geometry;

public class Vector2D extends Vector {
    public final double x;
    public final double y;

    public Vector2D(double x, double y) {
        super(x, y);
        this.x = x;
        this.y = y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D normalized() {
        double magnitude = magnitude();
        if (magnitude == 0) throw new IllegalArgumentException("Cannot normalize 0 vector");
        return new Vector2D(x / magnitude, y / magnitude);
    }

    public Vector2D plus(Vector2D other) {
        return new Vector2D(x + other.x, y + other.y);
    }

    public Vector2D minus(Vector2D other) {
        return new Vector2D(x - other.x, y - other.y);
    }

    public Vector2D times(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    public Vector2D div(double scalar) {
        return new Vector2D(x / scalar, y / scalar);
    }

    public Vector2D rotate(double angle) {
        return new Vector2D(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double dot(Vector2D other) {
        return x * other.x + y * other.y;
    }

    public Vector2D cross(Vector2D other) {
        return new Vector2D(x * other.y - y * other.x, x * other.x + y * other.y);
    }

    public Vector2D projectOnto(Vector2D other) {
        return other.times(dot(other) / other.dot(other));
    }

    public double theta() {
        return Math.atan2(y, x);
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    public static Vector2D polar(double radius, double angle) {
        return new Vector2D(radius * Math.cos(angle), radius * Math.sin(angle));
    }

    public static Vector2D cartesian(double x, double y) {
        return new Vector2D(x, y);
    }

    private static final Vector2D ZERO = new Vector2D(0, 0);
    private static final Vector2D I_HAT = new Vector2D(1, 0);
    private static final Vector2D J_HAT = new Vector2D(0, 1);

    public static Vector2D zero() {
        return ZERO;
    }

    public static Vector2D iHat() {
        return I_HAT;
    }

    public static Vector2D jHat() {
        return J_HAT;
    }

    public boolean isZero() {
        return x < 1e-9 && y < 1e-9;
    }

    public double quadraticForm(Matrix m) {
        return transform(m).dot(this);
    }

    public double distance(Vector2D other) {
        return Math.hypot(x - other.x, y - other.y);
    }
}
