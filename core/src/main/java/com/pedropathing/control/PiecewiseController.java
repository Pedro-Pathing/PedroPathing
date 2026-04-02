package com.pedropathing.control;

import java.util.TreeMap;

public class PiecewiseController implements Controller<Void> {

    private final TreeMap<Double, Controller<?>> controllers;

    public PiecewiseController(Controller<?> baseline) {
        this.controllers = new TreeMap<>();
        this.controllers.put(Double.NEGATIVE_INFINITY, baseline);
    }

    /**
     * Adds a controller to the piecewise controller. The controller will be used when the input is greater than or equal to the threshold and less than the next threshold.
     * @param threshold
     * @param controller
     * @return this
     */
    public PiecewiseController add(double threshold, Controller<?> controller) {
        controllers.put(threshold, controller);
        return this;
    }


}