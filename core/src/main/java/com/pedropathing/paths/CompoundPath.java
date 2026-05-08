package com.pedropathing.paths;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;

import java.util.*;

public class CompoundPath extends Path {
    private final NavigableMap<Double, Path> paths;
    private final Curve proxy = new ProxyCurve();

    CompoundPath(Path[] paths, Interpolator interpolator) {
        super(interpolator);
        if (paths.length == 0) throw new IllegalArgumentException("Compound path have at least one path.");

        length = Arrays.stream(paths).mapToDouble(Path::length).sum();

        this.paths = buildPathMap(paths, length);
    }

    private static NavigableMap<Double, Path> buildPathMap(Path[] paths, double length) {
        NavigableMap<Double, Path> pathMap = new TreeMap<>();
        double currentT = 0.0;
        for (Path path : paths) {
            pathMap.put(currentT, path);
            currentT += path.length() / length;
        }
        return pathMap;
    }

    private Path getPath(double t) {
        return paths.floorEntry(t).getValue();
    }

    private double getT(double t) {
        double startT = paths.floorKey(t);
        double endT = Objects.requireNonNullElse(paths.ceilingKey(t), 1.0);
        return (t - startT) / (endT - startT);
    }

    @Override
    double getHeading(double t) {
        if (interpolator != null) return interpolator.interpolate(t);
        else return getPath(t).getHeading(getT(t));
    }
}
