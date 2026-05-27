package com.pedropathing.controllers;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ControllerTest {
    @Test
    public void staticAndDynamicFeedforwardBehave() {
        Controller sf = Controller.staticFeedforward(2.0);
        assertEquals(2.0, sf.calculate(0, 1.0), 1e-9);
        assertEquals(-2.0, sf.calculate(0, -3.0), 1e-9);
        assertEquals(0.0, sf.calculate(0, 0.0), 1e-9);

        Controller df = Controller.dynamicFeedforward(0.5);
        assertEquals(2.0, df.calculate(4.0, 0.0), 1e-9);
    }

    @Test
    public void compositionPlusMinusTimesCombineControllers() {
        Controller a = Controller.staticFeedforward(1.0);
        Controller b = Controller.dynamicFeedforward(2.0);

        Controller sum = a.plus(b);
        // use a non-zero error so static feedforward contributes sign(error) * 1.0
        assertEquals(1.0 + 2.0 * 3.0, sum.calculate(3.0, 1.0), 1e-9);

        Controller diff = a.minus(b);
        assertEquals(1.0 - 2.0 * 3.0, diff.calculate(3.0, 1.0), 1e-9);

        Controller scaled = a.times(3.0);
        assertEquals(3.0 * 1.0, scaled.calculate(0.0, 1.0), 1e-9);
    }
}

