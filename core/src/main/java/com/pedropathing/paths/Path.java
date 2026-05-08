package com.pedropathing.paths;

import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.tvalue.TValue;
import lombok.experimental.Delegate;

public abstract class Path {
    @Delegate
    public final Curve curve;

    Path(Curve curve) {
        this.curve = curve;
    }

    abstract double getHeading(@TValue double t);
}
