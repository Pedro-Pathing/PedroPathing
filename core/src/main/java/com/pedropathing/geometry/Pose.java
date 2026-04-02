package com.pedropathing.geometry;

public class Pose {
    public final double x;
    public final double y;
    public final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = Angle.normalize(heading);
    }

    public Vector toVector() {
        return Vector.cartesian(x, y);
    }

    public Pose plus(Pose other) {
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose minus(Pose other) {
        return new Pose(x - other.x, y - other.y, heading - other.heading);
    }

    public Pose times(double scalar) {
        return new Pose(x * scalar, y * scalar, heading * scalar);
    }

    public Pose div(double scalar) {
        return new Pose(x / scalar, y / scalar, heading / scalar);
    }

    public String toString() {
        return "(" + x + ", " + y + ", " + heading + ")";
    }

    private static final Pose ZERO = new Pose(0, 0, 0);

    public static Pose zero() {
        return ZERO;
    }

    public Pose withX(double x) {
        return new Pose(x, y, heading);
    }

    public Pose withY(double y) {
        return new Pose(x, y, heading);
    }

    public Pose withHeading(double heading) {
        return new Pose(x, y, heading);
    }
}
