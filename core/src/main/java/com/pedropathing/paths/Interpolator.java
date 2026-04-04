package com.pedropathing.paths;

import com.pedropathing.math.Angle;
import com.pedropathing.math.Pose;

@FunctionalInterface
public interface Interpolator {
    double interpolate(Curve curve, double t);

    Interpolator tangent = (curve, t) -> curve.getTangent(t).theta();
    static Interpolator constant(double heading) { return (curve, t) -> Angle.normalize(heading); }
    static Interpolator linear(double start, double end, double endT) {
        double finalStart = Angle.normalize(start);
        double finalEnd = Angle.normalize(end);
        return (curve, t) -> {
            double clampedEndT = Angle.clamp(endT, 0.0001, 1);
            double tValue = Math.min(t / clampedEndT, 1.0);
            double deltaHeading = Angle.turnDirection(finalStart, finalEnd) * Angle.smallestDifference(finalStart, finalEnd);
            return Angle.normalize(finalStart + deltaHeading * tValue);
        };
    }
    static Interpolator linear(double start, double end) {
        return linear(start, end, 1);
    }
    static Interpolator facingPose(Pose pose) {
        return (curve, t) -> pose.toVector2D().minus(curve.get(t)).theta();
    }
}