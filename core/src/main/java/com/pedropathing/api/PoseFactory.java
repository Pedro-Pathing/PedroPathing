/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.api;

import com.pedropathing.math.Pose;
import java.util.function.DoubleUnaryOperator;

public final class PoseFactory {
    private final Operation operation;
    private final AngleUnit angleUnit;

    public PoseFactory(Operation operation, AngleUnit angleUnit) {
        this.operation = operation;
        this.angleUnit = angleUnit;
    }

    public PoseFactory(Operation operation, boolean useDegrees) {
        this(operation, useDegrees ? AngleUnit.DEGREES : AngleUnit.RADIANS);
    }

    /**
     * Creates a PoseFactory that uses degrees for heading.
     */
    public static PoseFactory degrees() {
        return new PoseFactory(Operation.IDENTITY, AngleUnit.DEGREES);
    }

    /**
     * Creates a PoseFactory that uses radians for heading.
     */
    public static PoseFactory radians() {
        return new PoseFactory(Operation.IDENTITY, AngleUnit.RADIANS);
    }

    /**
     * Creates a Pose with the given x, y, and heading.
     * The heading is interpreted in the unit specified by the PoseFactory (degrees or radians).
     * Any operations defined in the PoseFactory will be applied to the created Pose.
     */
    public Pose of(double x, double y, double heading) {
        return operation.apply(new Pose(x, y, angleUnit.toRadians(heading)));
    }

    public PoseFactory map(Operation operator) {
        return new PoseFactory(operation.andThen(operator), angleUnit);
    }

    /**
     * Returns a new PoseFactory that reflects the Pose across the vertical line at the specified
     * x-coordinate, mirroring the x-coordinate and the heading.
     */
    public PoseFactory mirrorX(double axis) {
        return map(pose -> pose.withX(2 * axis - pose.x()).withHeading(Math.PI - pose.heading()));
    }

    /**
     * Returns a new PoseFactory that reflects the Pose across the horizontal line at the specified
     * y-coordinate, mirroring the y-coordinate and the heading.
     */
    public PoseFactory mirrorY(double axis) {
        return map(pose -> pose.withY(2 * axis - pose.y()).withHeading(-pose.heading()));
    }

    public PoseFactory mirrorAroundPoint(double centerX, double centerY) {
        return map(pose ->
                pose.withX(2 * centerX - pose.x()).withY(2 * centerY - pose.y()).withHeading(pose.heading() + Math.PI));
    }

    public PoseFactory mirrorAroundPoint(Pose center) {
        return mirrorAroundPoint(center.x(), center.y());
    }

    public PoseFactory mapX(DoubleUnaryOperator operator) {
        return map(pose -> pose.withX(operator.applyAsDouble(pose.x())));
    }

    public PoseFactory mapY(DoubleUnaryOperator operator) {
        return map(pose -> pose.withY(operator.applyAsDouble(pose.y())));
    }

    public PoseFactory mapHeading(DoubleUnaryOperator operator) {
        return map(pose ->
                pose.withHeading(angleUnit.toRadians(operator.applyAsDouble(angleUnit.fromRadians(pose.heading())))));
    }

    public enum AngleUnit {
        DEGREES {
            @Override
            public double toRadians(double heading) {
                return Math.toRadians(heading);
            }

            @Override
            public double fromRadians(double radians) {
                return Math.toDegrees(radians);
            }
        },
        RADIANS {
            @Override
            public double toRadians(double heading) {
                return heading;
            }

            @Override
            public double fromRadians(double radians) {
                return radians;
            }
        };

        public abstract double toRadians(double heading);

        public abstract double fromRadians(double radians);
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
