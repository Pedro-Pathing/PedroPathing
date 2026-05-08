package com.pedropathing.math;

import lombok.Value;
import lombok.With;

@Value
@With
public class Pose {
    private static final Pose ZERO = new Pose(0, 0, 0);
    double x;
    double y;
    double heading;

    public static Pose zero() {
        return ZERO;
    }

    public Vector2D toVector2D() {
        return Vector2D.cartesian(x, y);
    }

    public Matrix toMatrix() {
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        return new Matrix(new double[][]{
                {cos, -sin, x},
                {sin, cos, y},
                {0.0, 0.0, 1.0}
        });
    }

    public Pose exp(Velocity velocity, double time) {
        return new Pose(x + velocity.vx * time, y + velocity.vy * time, heading + velocity.omega * time);
    }

    public Pose exp(Twist twist, double time) {
        if (twist.omega() < 1e-9) return exp(twist.toVelocity(heading), time);
        double theta = twist.omega() * time;
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        Vector2D localDeltas = Vector2D.cartesian((sin * twist.vx() - (1 - cos) * twist.vy()) / twist.omega(),
                ((1 - cos) * twist.vx() + sin * twist.vy()) / twist.omega());
        Vector2D globalDeltas = localDeltas.transform(Matrix.rotation(heading)); //TODO: Implement Matrix2D or smth
        return new Pose(x + globalDeltas.x(), y + globalDeltas.y(), heading + theta);
    }

    public Pose compose(Pose other) {
        Vector2D translationDeltas = other.toVector2D().rotate(heading);
        return new Pose(x + translationDeltas.x(), y + translationDeltas.y(), heading + other.heading);
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
    public double distance(Pose other) {
        return Math.hypot(x - other.x, y - other.y);
    }
}
