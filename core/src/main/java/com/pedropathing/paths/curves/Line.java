package com.pedropathing.paths.curves;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.tvalue.TValue;

public class Line implements Curve {
    private final Vector2D start;
    private final Vector2D end;
    private final double length;
    private final Vector2D tangent;

    public Line(Vector2D start, Vector2D end) {
        this.start = start;
        this.end = end;
        length = start.distance(end);
        tangent = end.minus(start).normalized();
    }

    @Override
    public Vector2D startPoint() {
        return start;
    }

    @Override
    public Vector2D endPoint() {
        return end;
    }

    @Override
    public Vector2D get(@TValue double t) {
        return start.plus(end.minus(start).times(t));
    }

    @Override
    public Vector2D tangent(@TValue double t) {
        return tangent;
    }

    @Override
    public double curvature(@TValue double t) {
        return 0;
    }

    @Override
    public double closestT(Vector2D position) {
        Vector2D BA = end.minus(start);
        Vector2D PA = position.minus(start);

        return MathFunctions.clamp(BA.dot(PA) / Math.pow(BA.magnitude(), 2), 0, 1);
    }

    @Override
    public double length() {
        return length;
    }
}
