package com.pedropathing.math;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class Vector2DTest {
	@Test
	public void polarAndUnitAreConsistent() {
		Vector2D v = Vector2D.polar(2.0, Math.PI / 4);
		assertEquals(2.0, v.magnitude(), 1e-9);
		Vector2D unit = Vector2D.unit(Math.PI / 4);
		assertEquals(1.0, unit.magnitude(), 1e-9);
	}

	@Test
	public void normalizeAndRotate() {
		Vector2D v = Vector2D.cartesian(3, 4);
		Vector2D n = v.normalized();
		assertEquals(1.0, n.magnitude(), 1e-6);

		Vector2D r = Vector2D.cartesian(1, 0).rotate(Math.PI / 2);
		assertEquals(0.0, r.x(), 1e-6);
		assertEquals(1.0, r.y(), 1e-6);
	}

	@Test
	public void dotAndProjection() {
		Vector2D a = Vector2D.cartesian(1, 2);
		Vector2D b = Vector2D.cartesian(2, 0);
		assertEquals(2.0, a.dot(b), 1e-9);

		Vector2D proj = a.projectOnto(b);
		// projection onto x-axis should zero out y
		assertEquals(0.0, proj.y(), 1e-9);
	}

	@Test
	public void angleAndDistance() {
		Vector2D a = Vector2D.cartesian(1, 0);
		Vector2D b = Vector2D.cartesian(0, 1);
		assertEquals(Math.PI / 2, a.angleTo(b), 1e-6);
		assertEquals(Math.sqrt(2), a.distance(b), 1e-9);
	}

	@Test
	public void hadamardAndPerpLeft() {
		Vector2D a = Vector2D.cartesian(2, 3);
		Vector2D b = Vector2D.cartesian(4, 5);
		Vector2D h = a.hadamardProduct(b);
		assertEquals(8.0, h.x(), 1e-9);
		assertEquals(15.0, h.y(), 1e-9);

		Vector2D p = a.perpendicularLeft();
		assertEquals(-3.0, p.x(), 1e-9);
		assertEquals(2.0, p.y(), 1e-9);
	}
}


