package com.pedropathing.paths.interpolator;

import com.pedropathing.paths.tvalue.TValue;
import com.pedropathing.paths.curves.Curve;

import java.util.NavigableMap;
import java.util.TreeMap;

public class PiecewiseInterpolator implements Interpolator {
    private final NavigableMap<Double, Interpolator> interpolators = new TreeMap<>();
    private double greatestT = 0.0;
    PiecewiseInterpolator() {
    }

    public PiecewiseInterpolator add(@TValue double t, Interpolator interpolator) {
        if (t < greatestT)
            throw new IllegalArgumentException("t was " + t + " but  must be greater than " + greatestT + ", the greatest t already defined.");
        if (t > 1.0) throw new IllegalArgumentException("t must be less than or equal to 1.0.");
        greatestT = t;
        interpolators.put(t, interpolator);
        return this;
    }

    @Override
    public double interpolate(Curve curve, @TValue double t) {
        if (greatestT < 1.0)
            throw new IllegalStateException("piecewise interpolation must be fully defined before interpolating.");
        if (t < 0.0 || t > 1.0) throw new IllegalArgumentException("t must be between 0.0 and 1.0.");
        return interpolators.ceilingEntry(t).getValue().interpolate(curve, t);
    }
}
