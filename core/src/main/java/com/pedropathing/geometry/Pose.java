package com.pedropathing.geometry;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

/**
 * Represents a pose in 2D space, consisting of an \`x\` and \`y\` position and a heading (orientation).
 * The pose can be associated with a specific coordinate system.
 *
 * <p>
 * Provides methods for pose arithmetic, coordinate system conversion, and utility functions such as
 * distance calculation and conversion between polar and cartesian coordinates.
 * </p>
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @author BeepBot99
 * @version 1.0, 4/2/2024
 */
public final class Pose implements FuturePose {
    private final double x;
    private final double y;
    private final double heading;
    private final CoordinateSystem coordinateSystem;

    /**
     * Constructs a pose with the specified position, heading, and coordinate system.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param heading the heading in radians
     * @param coordinateSystem the coordinate system
     */
    public Pose(double x, double y, double heading, CoordinateSystem coordinateSystem) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.coordinateSystem = coordinateSystem;
    }

    /**
     * Constructs a pose with the specified position and heading, using the default Pedro coordinate system.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param heading the heading in radians
     */
    public Pose(double x, double y, double heading) {
        this(x, y, heading, PedroCoordinates.INSTANCE);
    }

    /**
     * Constructs a pose with the specified position and a heading of 0, using the default Pedro coordinate system.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     */
    public Pose(double x, double y) {
        this(x, y, 0);
    }

    /**
     * Constructs a pose at the origin (0, 0) with heading 0, using the default Pedro coordinate system.
     */
    public Pose() {
        this(0,0,0);
    }

    /**
     * Returns the x-coordinate of the pose.
     *
     * @return the x-coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y-coordinate of the pose.
     *
     * @return the y-coordinate
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the heading of the pose in radians.
     *
     * @return the heading in radians
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Returns the coordinate system of the pose.
     *
     * @return the coordinate system
     */
    public CoordinateSystem getCoordinateSystem() {
        return coordinateSystem;
    }

    /**
     * Returns a new pose with the specified x-coordinate, keeping other values the same.
     *
     * @param x the new x-coordinate
     * @return a new pose with updated x
     */
    public Pose withX(double x) {
        return new Pose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns a new pose with the specified y-coordinate, keeping other values the same.
     *
     * @param y the new y-coordinate
     * @return a new pose with updated y
     */
    public Pose withY(double y) {
        return new Pose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns a new pose with the specified heading, keeping other values the same.
     *
     * @param heading the new heading in radians
     * @return a new pose with updated heading
     */
    public Pose withHeading(double heading) {
        return new Pose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns the position of the pose as a \`Vector\`.
     *
     * @return a vector representing the position
     */
    public Vector getAsVector() {
        Vector vector = new Vector();
        vector.setOrthogonalComponents(x, y);
        return vector;
    }

    /**
     * Returns the heading of the pose as a unit vector.
     *
     * @return a unit vector in the direction of the heading
     */
    public Vector getHeadingAsUnitVector() {
        return new Vector(1, heading);
    }

    /**
     * Adds another pose to this pose, converting coordinate systems if necessary.
     *
     * @param other the pose to add
     * @return the resulting pose
     */
    public Pose plus(Pose other) {
        Pose inCurrentCoordinates = coordinateSystem == other.coordinateSystem ? other :
                other.getAsCoordinateSystem(
                        coordinateSystem);
        return new Pose(x + inCurrentCoordinates.x,
                y + inCurrentCoordinates.y,
                heading + inCurrentCoordinates.heading,
                coordinateSystem);
    }

    /**
     * Subtracts another pose from this pose, converting coordinate systems if necessary.
     *
     * @param other the pose to subtract
     * @return the resulting pose
     */
    public Pose minus(Pose other) {
        Pose inCurrentCoordinates = coordinateSystem == other.coordinateSystem ? other :
                other.getAsCoordinateSystem(
                        coordinateSystem);
        return new Pose(x - inCurrentCoordinates.x,
                y - inCurrentCoordinates.y,
                heading - inCurrentCoordinates.heading,
                coordinateSystem);
    }

    /**
     * Multiplies the pose by a scalar.
     * This scales x, y, and heading by the scalar value, but does not change the coordinate system.
     *
     * @param scalar the scalar value
     * @return the resulting pose
     */
    public Pose times(double scalar) {
        return new Pose(
                x * scalar,
                y * scalar,
                heading * scalar,
                coordinateSystem
        );
    }

    /**
     * This scales the current Pose by a given scale factor.
     * The heading or coordinate system is not affected.
     *
     * @param scalar the scale factor
     * @return returns the scaled Pose
     */
    public Pose scale(double scalar) {
        return new Pose(getX() * scalar, getY() * scalar, getHeading(), coordinateSystem);
    }

    /**
     * Divides the pose by a scalar.
     *
     * @param scalar the scalar value
     * @return the resulting pose
     */
    public Pose div(double scalar) {
        return new Pose(
                x / scalar,
                y / scalar,
                heading / scalar,
                coordinateSystem
        );
    }

    /**
     * Returns the negation of this pose.
     *
     * @return a pose with all components negated
     */
    public Pose unaryMinus() {
        return new Pose(-x, -y, -heading, coordinateSystem);
    }

    /**
     * This returns a Pose that is the linear combination of the current pose and another pose.
     * @param other the other pose
     * @param scaleThis the coefficient for the current pose
     * @param scaleOther the second coefficient for the other pose
     * @return the resulting pose
     */
    public Pose linearCombination(Pose other, double scaleThis, double scaleOther) {
        return times(scaleThis).plus(other.times(scaleOther));
    }

    /**
     * Checks if this pose is approximately equal to another pose within a given accuracy.
     *
     * @param other the other pose
     * @param accuracy the allowed difference
     * @return true if poses are approximately equal, false otherwise
     */
    public boolean roughlyEquals(Pose other, double accuracy) {
        return MathFunctions.roughlyEquals(x, other.x, accuracy) &&
                MathFunctions.roughlyEquals(y, other.y, accuracy) &&
                MathFunctions.roughlyEquals(MathFunctions.getSmallestAngleDifference(heading,
                        other.heading), 0, accuracy);
    }

    /**
     * Calculates the Euclidean distance from this pose to another pose.
     *
     * @param other the other pose
     * @return the distance between the two poses
     */
    public double distanceFrom(Pose other) {
        return Math.sqrt(Math.pow(other.x - x, 2) + Math.pow(other.y - y, 2));
    }

    /**
     * Calculates the squared Euclidean distance from this pose to another pose.
     * @param other the other pose
     * @return the squared distance between the two poses
     */
    public double distSquared(Pose other) {
        double xDist = other.x - x;
        double yDist = other.y - y;
        return xDist * xDist + yDist * yDist;
    }

    /**
     * This rotates this pose by the given theta
     *
     * @param theta the angle to rotate by.
     * @param rotateHeading whether to adjust the Pose heading too.
     * @return the rotated Pose.
     */
    public Pose rotate(double theta, boolean rotateHeading) {
        double x = getX() * Math.cos(theta) - getY() * Math.sin(theta);
        double y = getX() * Math.sin(theta) + getY() * Math.cos(theta);
        double heading = rotateHeading ? MathFunctions.normalizeAngle(getHeading() + theta) : getHeading();

        return new Pose(x, y, heading, coordinateSystem);
    }

    /**
     * This mirrors this pose across x = 72 in Pedro coordinates. This will return a new Pose in Pedro coordinates.
     * @return the mirrored Pose.
     */
    public Pose mirror() {
        Pose k = getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        return new Pose(144 - k.getX(), k.getY(), MathFunctions.normalizeAngle(Math.PI - k.getHeading()), PedroCoordinates.INSTANCE);
    }

    /**
     * Converts this pose to the specified coordinate system.
     *
     * @param coordinateSystem the target coordinate system
     * @return the pose in the target coordinate system
     */
    public Pose getAsCoordinateSystem(CoordinateSystem coordinateSystem) {
        return coordinateSystem.convertFromPedro(this.coordinateSystem.convertToPedro(this));
    }

    /**
     * Converts polar coordinates to cartesian coordinates.
     *
     * @param r the radius
     * @param theta the angle in radians
     * @return an array \`[x, y]\` representing the cartesian coordinates
     */
    public static double[] polarToCartesian(double r, double theta) {
        return new double[]{r * Math.cos(theta), r * Math.sin(theta)};
    }

    /**
     * Converts cartesian coordinates to polar coordinates.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @return an array \`[r, theta]\` where r is the radius and theta is the angle in radians
     */
    public static double[] cartesianToPolar(double x, double y) {
        return new double[] {Math.sqrt(x * x + y * y), Math.atan2(y, x)};
    }

    /**
     * Returns a new pose with the specified heading, keeping x and y the same.
     *
     * @param heading the new heading in radians
     * @return a new pose with updated heading
     */
    public Pose setHeading(double heading) {
        return new Pose(x, y, heading);
    }

    /** Returns a new pose with the same coordinate system, keeping x, y, and heading. */
    public Pose copy() {
        return new Pose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns a string representation of the pose in the format (x, y, heading in degrees).
     *
     * @return a string representation of the pose
     */
    @Override
    public String toString() {
        return "(" + getX() + ", " + getY() + ", " + Math.toDegrees(getHeading()) + ")";
    }

    /**
     * A utility method to distinguish a Pose from a FuturePose.
     * @return true, indicating that this Pose is initialized.
     */
    @Override
    public boolean initialized() {
        return true;
    }

    @Override
    public Pose getPose() {
        return this;
    }
}