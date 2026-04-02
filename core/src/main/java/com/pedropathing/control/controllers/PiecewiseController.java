package com.pedropathing.control.controllers;

import java.util.TreeMap;
import java.util.stream.Stream;

public class PiecewiseController implements Controller {

    private final TreeMap<Double, Controller> controllers;

    public PiecewiseController(Controller baseline) {
        this.controllers = new TreeMap<>();
        this.controllers.put(Double.NEGATIVE_INFINITY, baseline);
    }

    /**
     * Adds a controller to the piecewise controller.
     * The controller will be used when the input is greater than or equal to the threshold and less than the next threshold.
     *
     * @return this
     */
    public PiecewiseController add(double threshold, Controller controller) {
        controllers.put(threshold, controller);
        return this;
    }

    @Override
    public double calculate(double error) {
        return controllers.get(controllers.floorKey(error)).calculate(error);
    }

    @Override
    public void reset() {
        controllers.values().forEach(Controller::reset);
    }
}