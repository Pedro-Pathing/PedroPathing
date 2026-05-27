package com.pedropathing.controllers;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class PiecewiseControllerTest {
    @Test
    public void piecewiseSelectsCorrectControllerByThreshold() {
        Controller baseline = Controller.staticFeedforward(1.0);
        PiecewiseController pc = Controller.piecewise(baseline)
                .add(0.0, Controller.staticFeedforward(2.0))
                .add(1.0, Controller.staticFeedforward(3.0));

        // error < 0 -> baseline (sign preserved)
        assertEquals(-1.0, pc.calculate(0.0, -1.0), 1e-9);
        // error in [0.0,1.0) -> controller at 0.0
        assertEquals(2.0, pc.calculate(0.0, 0.5), 1e-9);
        // error >= 1.0 -> controller at 1.0
        assertEquals(3.0, pc.calculate(0.0, 2.0), 1e-9);
    }
}

