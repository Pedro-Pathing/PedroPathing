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

    public Vector2D toVector() {
        return Vector2D.cartesian(x, y);
    }

    public Matrix toMatrix() {
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        return new Matrix(new double[][]{
                {cos, -sin,   x},
                {sin,  cos,   y},
                {0.0,  0.0, 1.0}
        });
    }

    public Pose integrate(Velocity velocity, double time) {
        return new Pose(x + velocity.vx * time, y + velocity.vy * time, heading + velocity.omega * time);
    }

    public Pose integrate(Twist twist, double time) {
        if (twist.omega < 1e-9) return integrate(twist.toVelocity(heading), time);
        double theta = twist.omega * time;
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        Vector2D localDeltas = new Vector2D((sin * twist.vx - (1 - cos) * twist.vy) / twist.omega,
                ((1 - cos) * twist.vx + sin * twist.vy) / twist.omega);
        Vector2D globalDeltas = (Vector2D) localDeltas.transform(Matrix.rotation(heading)); //TODO: Implement Matrix2D or smth
        return new Pose(x + globalDeltas.x, y + globalDeltas.y, heading + theta);
    }

    public Pose compose(Pose other) {
        Vector2D translationDeltas = other.toVector().rotate(heading);
        return new Pose(x + translationDeltas.x, y + translationDeltas.y, heading + other.heading);
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

    public double distance(Pose other) {
        return Math.hypot(x - other.x, y - other.y);
    }
}
