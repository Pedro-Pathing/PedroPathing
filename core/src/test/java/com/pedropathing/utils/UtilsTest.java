package com.pedropathing.utils;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class UtilsTest {
    @Test
    public void controlHelpersRespectPowerBudgets() {
        assertEquals(3.0, Control.getRemainingMagnitude(5.0, 4.0), 1e-9);
        assertEquals(2.5, Control.allocatePower(2.5, 3.0), 1e-9);
        assertEquals(-3.0, Control.allocatePower(-4.0, 3.0), 1e-9);

        double[] powers = {2.0, -4.0, 1.0};
        Control.desaturate(powers);
        assertArrayEquals(new double[] {0.5, -1.0, 0.25}, powers, 1e-9);
    }

    @Test
    public void controlCosineScaleAndBrakeClampBehaveAsExpected() {
        assertEquals(1.0, Control.cosineScale(0.0, 2.0), 1e-9);
        assertEquals(0.0, Control.cosineScale(2.0, 2.0), 1e-9);
        assertEquals(-0.5, Control.clampBrakingPower(-0.75, 1.0, 0.5), 1e-9);
        assertEquals(-0.5, Control.clampBrakingPower(-1.0, 1.0, 0.5), 1e-9);
        assertEquals(0.8, Control.clampBrakingPower(0.8, 1.0, 0.5), 1e-9);
    }

    @Test
    public void angleHelpersNormalizeAndChooseDirection() {
        assertEquals(3 * Math.PI / 2, Angle.normalize(-Math.PI / 2), 1e-9);
        assertEquals(Math.PI / 2, Angle.normalize(5 * Math.PI / 2), 1e-9);
        assertEquals(-Math.PI, Angle.normalizeSigned(Math.PI), 1e-9);
        assertEquals(Math.PI / 2, Angle.smallestDifference(0.0, 3 * Math.PI / 2), 1e-9);
        assertEquals(1.0, Angle.turnDirection(0.0, Math.PI / 2), 1e-9);
        assertEquals(-1.0, Angle.turnDirection(0.0, 3 * Math.PI / 2), 1e-9);
    }

    @Test
    public void quadraticSolverAndClampWork() {
        Pair<Double, Double> roots = Utils.solveQuadratic(1.0, -5.0, 6.0);
        assertTrue((Math.abs(roots.first() - 2.0) < 1e-9 && Math.abs(roots.second() - 3.0) < 1e-9)
                || (Math.abs(roots.first() - 3.0) < 1e-9 && Math.abs(roots.second() - 2.0) < 1e-9));

        assertEquals(2.0, Utils.clamp(2.0, 1.0, 3.0), 1e-9);
        assertEquals(1.0, Utils.clamp(0.0, 1.0, 3.0), 1e-9);
        assertEquals(3.0, Utils.clamp(4.0, 1.0, 3.0), 1e-9);
    }
}

