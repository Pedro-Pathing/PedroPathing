package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the HeadingInterpolatorBuilder class. This class makes it easier to create piecewise
 * heading interpolations, so you don't have to use verbose nested constructor calls with
 * HeadingInterpolator.PiecewiseNode. The builder tracks the current t-value, allowing segments
 * to be chained automatically without repeatedly specifying start t-values.
 *
 * <p>
 * Example usage:
 * <pre><code>
 * // Without builder:
 * follower.pathBuilder()
 *     .addPath(curve)
 *     .setHeadingInterpolation(HeadingInterpolator.piecewise(
 *         HeadingInterpolator.PiecewiseNode.linear(0, .2, startHeading, midHeading),
 *         new HeadingInterpolator.PiecewiseNode(.2, .6, HeadingInterpolator.constant(midHeading)),
 *         HeadingInterpolator.PiecewiseNode.linear(.6, 1, midHeading, endHeading)
 *     ))
 *     .build();
 *
 * // With builder (explicit t-values):
 * follower.pathBuilder()
 *     .addPath(curve)
 *     .setHeadingInterpolation(new HeadingInterpolatorBuilder()
 *         .linear(0, .2, startHeading, midHeading)
 *         .constant(.2, .6, midHeading)
 *         .linear(.6, 1, midHeading, endHeading)
 *         .build())
 *     .build();
 *
 * // With builder (auto-chaining):
 * follower.pathBuilder()
 *     .addPath(curve)
 *     .setHeadingInterpolation(new HeadingInterpolatorBuilder()
 *         .linear(0, .2, startHeading, midHeading)
 *         .constant(.6, midHeading)        // automatically uses .2 as startT
 *         .linear(1, midHeading, endHeading) // automatically uses .6 as startT
 *         .build())
 *     .build();
 * </code></pre>
 *
 * @author Maxwell Tham - 6165 MSET Cuttlefish
 * @author Eric Woo-Shem - 6165 MSET Cuttlefish
 * @version 1.0, 3/24/2026
 */
public class HeadingInterpolatorBuilder {
    private final List<HeadingInterpolator.PiecewiseNode> nodes;
    private double currentT = 0;

    /**
     * This is the constructor for the HeadingInterpolatorBuilder class.
     * The HeadingInterpolatorBuilder allows for easier construction of piecewise heading
     * interpolations.
     */
    public HeadingInterpolatorBuilder() {
        this.nodes = new ArrayList<>();
    }

    /**
     * This adds a linear heading interpolation segment from startT to endT.
     * The robot will transition from the start heading to the end heading over the segment.
     *
     * @param startT The starting t-value on the Path for when the interpolation begins.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param startHeadingRad The start of the linear heading interpolation.
     * @param endHeadingRad The end of the linear heading interpolation.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder linear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
        nodes.add(HeadingInterpolator.PiecewiseNode.linear(startT, endT, startHeadingRad, endHeadingRad));
        currentT = endT;
        return this;
    }

    /**
     * This adds a linear heading interpolation segment from the current t-value to endT.
     * The current t-value is automatically set to the end t-value of the previous segment.
     *
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param startHeadingRad The start of the linear heading interpolation.
     * @param endHeadingRad The end of the linear heading interpolation.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder linear(double endT, double startHeadingRad, double endHeadingRad) {
        return linear(currentT, endT, startHeadingRad, endHeadingRad);
    }

    /**
     * This adds a reversed linear heading interpolation segment from startT to endT.
     * The robot will transition from the start heading to the end heading over the segment.
     *
     * @param startT The starting t-value on the Path for when the interpolation begins.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param startHeadingRad The start of the reversed linear heading interpolation.
     * @param endHeadingRad The end of the reversed linear heading interpolation.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder reversedLinear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
        nodes.add(HeadingInterpolator.PiecewiseNode.reversedLinear(startT, endT, startHeadingRad, endHeadingRad));
        currentT = endT;
        return this;
    }

    /**
     * This adds a reversed linear heading interpolation segment from the current t-value to endT.
     * The current t-value is automatically set to the end t-value of the previous segment.
     *
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param startHeadingRad The start of the reversed linear heading interpolation.
     * @param endHeadingRad The end of the reversed linear heading interpolation.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder reversedLinear(double endT, double startHeadingRad, double endHeadingRad) {
        return reversedLinear(currentT, endT, startHeadingRad, endHeadingRad);
    }

    /**
     * This adds a constant heading segment from startT to endT.
     * The robot will maintain the specified heading over the segment.
     *
     * @param startT The starting t-value on the Path for when the interpolation begins.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param headingRad The constant heading specified.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder constant(double startT, double endT, double headingRad) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.constant(headingRad)));
        currentT = endT;
        return this;
    }

    /**
     * This adds a constant heading segment from the current t-value to endT.
     * The current t-value is automatically set to the end t-value of the previous segment.
     *
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param headingRad The constant heading specified.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder constant(double endT, double headingRad) {
        return constant(currentT, endT, headingRad);
    }

    /**
     * This adds a tangent heading segment from startT to endT.
     * The robot will face the direction of the path over the segment.
     *
     * @param startT The starting t-value on the Path for when the interpolation begins.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder tangent(double startT, double endT) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.tangent));
        currentT = endT;
        return this;
    }

    /**
     * This adds a tangent heading segment from the current t-value to endT.
     * The robot will face the direction of the path over the segment.
     * The current t-value is automatically set to the end t-value of the previous segment.
     *
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder tangent(double endT) {
        return tangent(currentT, endT);
    }

    /**
     * This adds a facing point heading segment from startT to endT.
     * The robot will always be facing the given point while following the segment.
     *
     * @param startT The starting t-value on the Path for when the interpolation begins.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param x The x-coordinate of the point to face.
     * @param y The y-coordinate of the point to face.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder facingPoint(double startT, double endT, double x, double y) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.facingPoint(x, y)));
        currentT = endT;
        return this;
    }

    /**
     * This adds a facing point heading segment from the current t-value to endT.
     * The robot will always be facing the given point while following the segment.
     * The current t-value is automatically set to the end t-value of the previous segment.
     *
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param x The x-coordinate of the point to face.
     * @param y The y-coordinate of the point to face.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder facingPoint(double endT, double x, double y) {
        return facingPoint(currentT, endT, x, y);
    }

    /**
     * This adds a facing point heading segment from startT to endT.
     * The robot will always be facing the given Pose while following the segment.
     *
     * @param startT The starting t-value on the Path for when the interpolation begins.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param pose The Pose to face.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder facingPoint(double startT, double endT, Pose pose) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.facingPoint(pose)));
        currentT = endT;
        return this;
    }

    /**
     * This adds a facing point heading segment from the current t-value to endT.
     * The robot will always be facing the given Pose while following the segment.
     * The current t-value is automatically set to the end t-value of the previous segment.
     *
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param pose The Pose to face.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder facingPoint(double endT, Pose pose) {
        return facingPoint(currentT, endT, pose);
    }

    /**
     * This adds a custom HeadingInterpolator segment from startT to endT.
     *
     * @param startT The starting t-value on the Path for when the interpolation begins.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param interpolator The custom heading interpolator to use on this segment.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder custom(double startT, double endT, HeadingInterpolator interpolator) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, interpolator));
        currentT = endT;
        return this;
    }

    /**
     * This adds a custom HeadingInterpolator segment from the current t-value to endT.
     * The current t-value is automatically set to the end t-value of the previous segment.
     *
     * @param endT The ending t-value on the Path for when the interpolation ends.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @param interpolator The custom heading interpolator to use on this segment.
     * @return This returns itself with the updated data.
     */
    public HeadingInterpolatorBuilder custom(double endT, HeadingInterpolator interpolator) {
        return custom(currentT, endT, interpolator);
    }

    /**
     * This builds all the heading interpolation segments together into a HeadingInterpolator.
     *
     * @return This returns a HeadingInterpolator made of all the specified segments.
     * @throws IllegalStateException if no segments have been added.
     */
    public HeadingInterpolator build() {
        if (nodes.isEmpty()) {
            throw new IllegalStateException("Cannot build HeadingInterpolator with no segments. Add at least one segment.");
        }
        return HeadingInterpolator.piecewise(nodes.toArray(new HeadingInterpolator.PiecewiseNode[0]));
    }

    /**
     * This gets the current t-value, which is the end t-value of the last added segment.
     *
     * @return This returns the current t-value.
     */
    public double getCurrentT() {
        return currentT;
    }
}
