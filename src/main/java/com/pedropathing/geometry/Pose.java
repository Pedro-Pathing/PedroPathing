package com.pedropathing.geometry;

import static math.MathFunctions.rotatePose;

import androidx.annotation.NonNull;

import math.MathFunctions;

import math.Vector;

/**
 * This is the Pose class. It defines poses in 2D space. A Pose consists of
 * two coordinates defining a position and a third value for the heading, so basically just defining
 * any position and orientation the robot can be at, unless your robot can fly for whatever reason.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author BeepBot99
 * @version 1.0, 4/2/2024
 */
public final class Pose {
    private final double x;
    private final double y;
    private final double heading;
    private final CoordinateSystem coordinateSystem;

    public Pose(double x, double y, double heading, CoordinateSystem coordinateSystem) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.coordinateSystem = coordinateSystem;
    }

    public Pose(double x, double y, double heading) {
        this(x, y, heading, CoordinateSystems.PEDRO);
    }

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public CoordinateSystem getCoordinateSystem() {
        return coordinateSystem;
    }

    public Pose withX(double x) {
        return new Pose(x, y, heading, coordinateSystem);
    }

    public Pose withY(double y) {
        return new Pose(x, y, heading, coordinateSystem);
    }

    public Pose withHeading(double heading) {
        return new Pose(x, y, heading, coordinateSystem);
    }

    public Vector getAsVector() {
        Vector vector = new Vector();
        vector.setOrthogonalComponents(x, y);
        return vector;
    }

    public Vector getHeadingAsUnitVector() {
        return new Vector(1, heading);
    }

    public Pose plus(Pose other) {
        Pose inCurrentCoordinates = other.getAsCoordinateSystem(coordinateSystem);
        return new Pose(x + inCurrentCoordinates.x,
                y + inCurrentCoordinates.y,
                heading + inCurrentCoordinates.heading,
                coordinateSystem);
    }

    public Pose minus(Pose other) {
        Pose inCurrentCoordinates = coordinateSystem == other.coordinateSystem ? other :
                other.getAsCoordinateSystem(
                        coordinateSystem);
        return new Pose(x + inCurrentCoordinates.x,
                y + inCurrentCoordinates.y,
                heading + inCurrentCoordinates.heading,
                coordinateSystem);
    }

    public Pose times(double scalar) {
        return new Pose(
                x * scalar,
                y * scalar,
                heading * scalar,
                coordinateSystem
        );
    }

    public Pose div(double scalar) {
        return new Pose(
                x / scalar,
                y / scalar,
                heading / scalar,
                coordinateSystem
        );
    }

    public Pose unaryMinus() {
        return new Pose(-x, -y, -heading, coordinateSystem);
    }

    public boolean roughlyEquals(Pose other, double accuracy) {
        return MathFunctions.roughlyEquals(x, other.x, accuracy) &&
                MathFunctions.roughlyEquals(y, other.y, accuracy) &&
                MathFunctions.roughlyEquals(MathFunctions.getSmallestAngleDifference(heading,
                        other.heading), 0, accuracy);
    }

    public double distanceFrom(Pose other) {
        return Math.sqrt(Math.pow(other.x - x, 2) + Math.pow(other.y - y, 2));
    }

    public Pose getAsCoordinateSystem(CoordinateSystem coordinateSystem) {
        return coordinateSystem.convertFromFtcStandard(this.coordinateSystem.convertToFtcStandard(this));
    }

    public Pose getAsPedroCoordinates() {
        return getAsCoordinateSystem(CoordinateSystems.PEDRO);
    }

    public Pose getAsFtcStandardCoordinates() {
        return getAsCoordinateSystem(CoordinateSystems.FTC);
    }
}

