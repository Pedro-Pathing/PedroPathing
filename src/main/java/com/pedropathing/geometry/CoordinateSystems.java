package com.pedropathing.geometry;

import math.MathFunctions;

/**
 * An enum that contains the two default coordinate systems, {@link CoordinateSystems#FTC} and
 * {@link CoordinateSystems#PEDRO}. This enum implements the {@link CoordinateSystem} interface,
 * which specifies a way to convert to and from FTC standard coordinates.
 *
 * @author BeepBot99
 */
public enum CoordinateSystems implements CoordinateSystem {
    /**
     * The FTC standard coordinate system.
     * No conversions are performed, as this is the FTC standard coordinate system.
     */
    FTC {
        /**
         * Converts a {@link Pose} from FTC standard coordinates to FTC standard coordinates
         * Since they are the same coordinate system, no conversion is necessary.
         *
         * @param pose The {@link Pose} to convert, in FTC standard coordinates
         * @return The converted {@link Pose}, in FTC standard coordinates
         */
        @Override
        public Pose convertToFtcStandard(Pose pose) {
            return pose;
        }

        /**
         * Converts a {@link Pose} from FTC standard coordinates to FTC standard coordinates.
         * Since they are the same coordinate system, no conversion is necessary.
         *
         * @param pose The {@link Pose} to convert, in FTC standard coordinates
         * @return The converted {@link Pose}, in FTC standard coordinates.
         */
        @Override
        public Pose convertFromFtcStandard(Pose pose) {
            return pose;
        }
    },

    /**
     * The Pedro coordinate system.
     */
    PEDRO {
        /**
         * Converts a {@link Pose} from this coordinate system to FTC standard coordinates
         *
         * @param pose The {@link Pose} to convert, in this coordinate system
         * @return The converted {@link Pose}, in FTC standard coordinates
         */
        @Override
        public Pose convertToFtcStandard(Pose pose) {
            Pose normalizedPose = pose.minus(new Pose(72, 72));
            return MathFunctions.rotatePose(normalizedPose, -Math.PI / 2, true);
        }

        /**
         * Converts a {@link Pose} from FTC standard coordinates to this coordinate system
         *
         * @param pose The {@link Pose} to convert, in FTC standard coordinates
         * @return The converted {@link Pose}, in this coordinate system
         */
        @Override
        public Pose convertFromFtcStandard(Pose pose) {
            Pose rotatedPose = MathFunctions.rotatePose(pose, -Math.PI / 2, true);
            return rotatedPose.plus(new Pose(72, 72));
        }
    };
}
