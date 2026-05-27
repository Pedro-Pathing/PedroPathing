package com.pedropathing.math;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class CoreMathTest {
    @Test
    public void vectorSupportsArithmeticAndTransformations() {
        Vector a = new Vector(3.0, 4.0);
        Vector b = new Vector(1.0, -2.0);

        assertEquals(2, a.size());
        assertEquals(5.0, a.magnitude(), 1e-9);
        assertEquals(-5.0, a.dot(b), 1e-9);
        assertEquals(new Vector(4.0, 2.0).get(0), a.plus(b).get(0), 1e-9);
        assertEquals(new Vector(2.0, 6.0).get(1), a.minus(b).get(1), 1e-9);
        assertEquals(6.0, a.times(2.0).get(0), 1e-9);

        Vector rotated = new Vector(1.0, 0.0).transform(Matrix.rotation(Math.PI / 2));
        assertEquals(0.0, rotated.get(0), 1e-9);
        assertEquals(1.0, rotated.get(1), 1e-9);

        Vector2D asVector2D = a.toVector2D();
        assertEquals(3.0, asVector2D.x(), 1e-9);
        assertEquals(4.0, asVector2D.y(), 1e-9);
        assertEquals(new Vector(0.0, 1.0, 0.0).get(1), Vector.e(1, 3).get(1), 1e-9);
        assertEquals(0.0, Vector.zero(3).magnitude(), 1e-9);
    }

    @Test
    public void twistAndVelocityConvertConsistently() {
        Twist twist = new Twist(1.0, 2.0, 0.5);
        Velocity velocity = twist.toVelocity(Math.PI / 2);
        assertEquals(-2.0, velocity.vx, 1e-9);
        assertEquals(1.0, velocity.vy, 1e-9);
        assertEquals(0.5, velocity.omega, 1e-9);

        Twist roundTrip = velocity.toTwist(Math.PI / 2);
        assertEquals(twist.vx(), roundTrip.vx(), 1e-9);
        assertEquals(twist.vy(), roundTrip.vy(), 1e-9);
        assertEquals(twist.omega(), roundTrip.omega(), 1e-9);

        assertEquals(new Vector(1.0, 2.0, 0.5).get(2), twist.toVector().get(2), 1e-9);
        assertEquals(1.0, Twist.fromVector(Vector2D.cartesian(1.0, 2.0)).vx(), 1e-9);
        assertEquals(2.0, Velocity.fromVector(Vector2D.cartesian(1.0, 2.0)).vy, 1e-9);
    }

    @Test
    public void poseSupportsCompositionAndIntegration() {
        Pose origin = Pose.zero();
        assertSame(origin, Pose.zero());
        assertEquals(0.0, origin.distance(new Pose(0.0, 0.0, 0.0)), 1e-9);

        Pose pose = new Pose(1.0, 2.0, Math.PI / 2);
        assertEquals(1.0, pose.toVector2D().x(), 1e-9);
        assertEquals(2.0, pose.toVector2D().y(), 1e-9);

        Pose composed = pose.compose(new Pose(1.0, 0.0, Math.PI / 4));
        assertEquals(1.0, composed.x(), 1e-9);
        assertEquals(3.0, composed.y(), 1e-9);
        assertEquals(3 * Math.PI / 4, composed.heading(), 1e-9);

        Pose advanced = new Pose(1.0, 2.0, 0.0).exp(new Velocity(2.0, -1.0, 0.5), 2.0);
        assertEquals(5.0, advanced.x(), 1e-9);
        assertEquals(0.0, advanced.y(), 1e-9);
        assertEquals(1.0, advanced.heading(), 1e-9);

        Pose math = new Pose(2.0, 4.0, 6.0);
        assertEquals(new Pose(3.0, 5.0, 7.0), math.plus(new Pose(1.0, 1.0, 1.0)));
        assertEquals(new Pose(1.0, 3.0, 5.0), math.minus(new Pose(1.0, 1.0, 1.0)));
    }
}

