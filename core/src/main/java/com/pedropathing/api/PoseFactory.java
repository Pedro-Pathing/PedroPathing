/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.api;

import com.pedropathing.math.Pose;
import java.util.function.DoubleUnaryOperator;

public final class PoseFactory {
    private final Operation operation;
    private final boolean useDegrees;

    public PoseFactory(Operation operation, boolean useDegrees) {
        this.operation = operation;
        this.useDegrees = useDegrees;
    }

    /**
     * Creates a PoseFactory that uses degrees for heading.
     */
    public static PoseFactory degrees() {
        return new PoseFactory(Operation.IDENTITY, true);
    }

    /**
     * Creates a PoseFactory that uses radians for heading.
     */
    public static PoseFactory radians() {
        return new PoseFactory(Operation.IDENTITY, false);
    }

    /**
     * Creates a Pose with the given x, y, and heading.
     * The heading is interpreted in the unit specified by the PoseFactory (degrees or radians).
     * Any operations defined in the PoseFactory will be applied to the created Pose.
     */
    public Pose of(double x, double y, double heading) {
        return operation.apply(new Pose(x, y, useDegrees ? Math.toRadians(heading) : heading));
    }

    public PoseFactory map(Operation operator) {
        return new PoseFactory(operation.andThen(operator), useDegrees);
    }

    /**
     * Returns a new PoseFactory that mirrors the x-coordinate of the Pose across the specified axis and inverts the heading.
     */
    public PoseFactory mirrorX(double axis) {
        return map(pose -> pose.withX(2 * axis - pose.x()).withHeading(-pose.heading()));
    }

    /**
     * Returns a new PoseFactory that mirrors the y-coordinate of the Pose across the specified axis and keeps the heading unchanged.
     */
    public PoseFactory mirrorY(double axis) {
        return map(pose -> pose.withY(2 * axis - pose.y()).withHeading(pose.heading()));
    }

    public PoseFactory mapX(DoubleUnaryOperator operator) {
        return map(pose -> pose.withX(operator.applyAsDouble(pose.x())));
    }

    public PoseFactory mapY(DoubleUnaryOperator operator) {
        return map(pose -> pose.withY(operator.applyAsDouble(pose.y())));
    }

    public PoseFactory mapHeading(DoubleUnaryOperator operator) {
        return map(pose -> pose.withHeading(operator.applyAsDouble(pose.heading())));
    }

    @FunctionalInterface
    public interface Operation {
        Operation IDENTITY = pose -> pose;

        Pose apply(Pose pose);

        default Operation andThen(Operation operator) {
            return pose -> operator.apply(apply(pose));
        }
    }
}
