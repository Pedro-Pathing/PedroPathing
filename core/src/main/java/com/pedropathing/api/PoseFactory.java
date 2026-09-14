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
    private final AxesConvention axesConvention;

    public PoseFactory(Operation operation, AngleUnit angleUnit) {
        this(operation, angleUnit, AxesConvention.PEDRO);
    }

    public PoseFactory(Operation operation, boolean useDegrees) {
        this(operation, useDegrees ? AngleUnit.DEGREES : AngleUnit.RADIANS, AxesConvention.PEDRO);
    }

    public PoseFactory(Operation operation, AngleUnit angleUnit, AxesConvention axesConvention) {
        this.operation = operation;
        this.angleUnit = angleUnit;
        this.axesConvention = axesConvention;
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
     * Returns a new PoseFactory that interprets x, y, and heading in the given axes convention.
     */
    public PoseFactory convention(AxesConvention axesConvention) {
        return new PoseFactory(operation, angleUnit, axesConvention);
    }

    /**
     * Returns a new PoseFactory that interprets x, y, and heading in FIRST's axes convention.
     */
    public PoseFactory withFirstAxes() {
        return convention(AxesConvention.FIRST);
    }

    /**
     * Creates a Pose with the given x, y, and heading.
     * The heading is interpreted in the unit specified by the PoseFactory (degrees or radians),
     * and x, y, and heading are interpreted in the axes convention specified by the PoseFactory.
     * Any operations defined in the PoseFactory will be applied to the created Pose.
     */
    public Pose of(double x, double y, double heading) {
        return operation.apply(axesConvention.toPedro(x, y, angleUnit.toRadians(heading)));
    }

    public PoseFactory map(Operation operator) {
        return mapInPedro(pose -> {
            Pose converted = operator.apply(axesConvention.fromPedro(pose));
            return axesConvention.toPedro(converted.x(), converted.y(), converted.heading());
        });
    }

    /**
     * Returns a new PoseFactory that applies the given operation to the created Pose in Pedro's
     * axes convention, regardless of what convention the PoseFactory itself uses.
     */
    public PoseFactory mapInPedro(Operation operator) {
        return new PoseFactory(operation.andThen(operator), angleUnit, axesConvention);
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

    /**
     * The field coordinate convention a PoseFactory interprets x, y, and heading in.
     * Poses are always stored in {@link #PEDRO} coordinates.
     */
    public enum AxesConvention {
        /**
         * Pedro's convention: the origin is a field corner, +x points towards the red alliance,
         * +y is 90 degrees counter-clockwise of +x, and a heading of 0 faces +x.
         */
        PEDRO {
            @Override
            public Pose toPedro(double x, double y, double heading) {
                return new Pose(x, y, heading);
            }

            @Override
            public Pose fromPedro(Pose pose) {
                return pose;
            }
        },

        /**
         * The FIRST convention: the origin is the center of the field, +x points away from the
         * audience, and +y is 90 degrees counter-clockwise of +x.
         */
        FIRST {
            @Override
            public Pose toPedro(double x, double y, double heading) {
                return new Pose(FIELD_CENTER - y, FIELD_CENTER + x, heading + Math.PI / 2);
            }

            @Override
            public Pose fromPedro(Pose pose) {
                return new Pose(pose.y() - FIELD_CENTER, FIELD_CENTER - pose.x(), pose.heading() - Math.PI / 2);
            }
        };

        /**
         * Half the width of an FTC field, in inches. This is the offset between Pedro's
         * corner origin and the FIRST convention's field-center origin.
         */
        public static final double FIELD_CENTER = 70.75;

        /**
         * Converts x, y, and a heading in radians from this convention to Pedro's.
         */
        public abstract Pose toPedro(double x, double y, double heading);

        /**
         * Converts a Pose in Pedro's convention to this one. The returned Pose's heading is in
         * radians.
         */
        public abstract Pose fromPedro(Pose pose);
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
