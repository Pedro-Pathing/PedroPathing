package com.pedropathing.math;

import com.pedropathing.geometry.Pose;

/**
 * This is the Vector class. This class handles storing information about vectors, which are
 * basically Points but using polar coordinates as the default. The main reason this class exists
 * is because some vector math needs to be done in the Follower, and dot products and cross
 * products of Points just don't seem right. Also, there are a few more methods in here that make
 * using Vectors a little easier than using a Point in polar coordinates.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/11/2024
 */
public class Vector {
    /**
     * The magnitude (length) of the vector.
     */
    private double magnitude;
    /**
     * The direction (angle in radians) of the vector.
     */
    private double theta;
    /**
     * The x component of the vector in Cartesian coordinates.
     */
    private double xComponent;
    /**
     * The y component of the vector in Cartesian coordinates.
     */
    private double yComponent;

    /**
     * Constructs a new Vector with zero magnitude and direction.
     */
    public Vector() {
        setComponents(0, 0);
    }

    /**
     * Constructs a new Vector from a given Pose's x and y coordinates.
     *
     * @param pose the Pose object to extract x and y from
     */
    public Vector(Pose pose) {
        setOrthogonalComponents(pose.getX(), pose.getY());
    }

    /**
     * Constructs a new Vector with a specified magnitude and direction.
     *
     * @param magnitude the magnitude (length) of the vector
     * @param theta     the direction (angle in radians) of the vector
     */
    public Vector(double magnitude, double theta) {
        setComponents(magnitude, theta);
    }

    /**
     * Sets the vector's magnitude and direction (polar coordinates).
     * Updates the Cartesian components accordingly.
     *
     * @param magnitude the magnitude to set
     * @param theta     the direction (angle in radians) to set
     */
    public void setComponents(double magnitude, double theta) {
        double[] orthogonalComponents;
        if (magnitude < 0) {
            this.magnitude = -magnitude;
            this.theta = MathFunctions.normalizeAngle(theta + Math.PI);
        } else {
            this.magnitude = magnitude;
            this.theta = MathFunctions.normalizeAngle(theta);
        }
        orthogonalComponents = Pose.polarToCartesian(magnitude, theta);
        xComponent = orthogonalComponents[0];
        yComponent = orthogonalComponents[1];
    }

    /**
     * Sets only the magnitude of the vector, keeping the direction unchanged.
     *
     * @param magnitude the new magnitude
     */
    public void setMagnitude(double magnitude) {
        setComponents(magnitude, theta);
    }

    /**
     * Sets only the direction (theta) of the vector, keeping the magnitude unchanged.
     *
     * @param theta the new direction (angle in radians)
     */
    public void setTheta(double theta) {
        setComponents(magnitude, theta);
    }

    /**
     * Rotates the vector by a given angle (in radians).
     *
     * @param theta2 the angle to add to the current direction
     */
    public void rotateVector(double theta2) {
        setTheta(theta + theta2);
    }

    /**
     * Sets the vector's Cartesian components (x, y).
     * Updates the polar representation accordingly.
     *
     * @param xComponent the x component to set
     * @param yComponent the y component to set
     */
    public void setOrthogonalComponents(double xComponent, double yComponent) {
        double[] polarComponents;
        this.xComponent = xComponent;
        this.yComponent = yComponent;
        polarComponents = Pose.cartesianToPolar(xComponent, yComponent);
        magnitude = polarComponents[0];
        theta = polarComponents[1];
    }

    /**
     * This multiplies the current Vector by a scalar and returns the result as a Vector.
     *
     * @param scalar the scalar multiplying into the Vector.
     * @return returns the scaled Vector.
     */
    public Vector times(double scalar) {
        return new Vector(getMagnitude() * scalar, getTheta());
    }

    /**
     * This normalizes this Vector to be of magnitude 1, unless this Vector is the zero Vector.
     * In that case, it just returns back the zero Vector but with a different memory location.
     *
     * @return returns the normalized (or zero) Vector.
     */
    public Vector normalize() {
        if (getMagnitude() == 0) {
            return new Vector(0.0, getTheta());
        } else {
            return new Vector(getMagnitude() / Math.abs(getMagnitude()), getTheta());
        }
    }

    /**
     * This returns a Vector that is the sum of the this vector and the other input Vector.
     *
     * @param other the other Vector.
     * @return returns the sum of the Vectors.
     */
    public Vector plus(Vector other) {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(getXComponent() + other.getXComponent(), getYComponent() + other.getYComponent());
        return returnVector;
    }

    /**
     * This subtracts the other Vector from the current Vector and returns the result as a Vector.
     * Do note that order matters here.
     *
     * @param other the other Vector.
     * @return returns the second Vector subtracted from the first Vector.
     */
    public Vector minus(Vector other) {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(getXComponent() - other.getXComponent(), getYComponent() - other.getYComponent());
        return returnVector;
    }

    /**
     * This computes the dot product of the current Vector and the other Vector.
     *
     * @param other the Other Vector.
     * @return returns the dot product of the two Vectors.
     */
    public double dot(Vector other) {
        return getXComponent() * other.getXComponent() + getYComponent() * other.getYComponent();
    }

    /**
     * This computes the current Vector crossed with the other Vector, so a cross product.
     * Do note that order matters here.
     *
     * @param other the other Vector.
     * @return returns the cross product of the two Vectors.
     */
    public double cross(Vector other) {
        return getXComponent() * other.getYComponent() - getYComponent() * other.getXComponent();
    }

    /**
     * This returns a Vector that is the linear combination of the current vector and another vector.
     * @param other the other vector
     * @param scaleThis the coefficient for the current vector
     * @param scaleOther the second coefficient for the other vector
     * @return the resulting vector
     */
    public Vector linearCombination(Vector other, double scaleThis, double scaleOther) {
        return times(scaleThis).plus(other.times(scaleOther));
    }

    /**
     * Returns the magnitude (length) of the vector.
     *
     * @return the magnitude
     */
    public double getMagnitude() {
        return magnitude;
    }

    /**
     * Returns the direction (angle in radians) of the vector.
     *
     * @return the theta value
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Returns the x component of the vector.
     *
     * @return the x component
     */
    public double getXComponent() {
        return xComponent;
    }

    /**
     * Returns the y component of the vector.
     *
     * @return the y component
     */
    public double getYComponent() {
        return yComponent;
    }

    /**
     * Returns a copy of this vector.
     *
     * @return a new Vector with the same magnitude and direction
     */
    public Vector copy() {
        return new Vector(this.magnitude, this.theta);
    }

    /**
     * Returns a string representation of the vector, including magnitude, theta, x, and y components.
     *
     * @return a string describing the vector
     */
    @Override
    public String toString() {
        return "Vector{" +
                "magnitude=" + magnitude +
                ", theta=" + theta +
                ", xComponent=" + xComponent +
                ", yComponent=" + yComponent +
                '}';
    }

    /**
     * Transforms the vector by multiplying it with a matrix
     *
     * @param matrix the matrix transformation
     * @return the resulting vector after applying the transformation
     */
    public Vector transform(Matrix matrix) {
        double[] multiply = matrix.multiply(new Matrix(new double[][]{{xComponent}, {yComponent}})).getCol(0);
        return new Vector(multiply[0], multiply[1]);
    }

    /**
     * Projects this vector onto another vector.
     *
     * @param other the vector to project onto
     * @return a new vector that is the projection of this vector onto the other vector
     */
    public Vector projectOnto(Vector other) {
        if (other.getMagnitude() == 0) return new Vector();
        double scale = this.dot(other) / other.dot(other);
        return other.times(scale);
    }

    /**
     * Multiplies this vector by -1
     * @return the resulting vector
     */
    public Vector flip() {
        return new Vector(magnitude, theta + Math.PI);
    }
}