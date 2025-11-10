package com.pedropathing.ftc;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;

/**
 * An enum that contains the FTC standard coordinate system.
 * This enum implements the {@link CoordinateSystem} interface, which specifies a way to convert to and from FTC standard coordinates.
 *
 * @author BeepBot99
 */
public enum InvertedFTCCoordinates implements CoordinateSystem {
    INSTANCE;

    /**
     * Converts a {@link Pose} to this coordinate system from Pedro coordinates
     *
     * @param pose The {@link Pose} to convert, in the Pedro coordinate system
     * @return The converted {@link Pose}, in FTC standard coordinates
     */
    @Override
    public Pose convertFromPedro(Pose pose) {
        Pose normalizedPose = pose.minus(new Pose(72, 72));
        return normalizedPose.rotate(Math.PI / 2, true);
    }

    /**
     * Converts a {@link Pose} to Pedro coordinates from this coordinate system
     *
     * @param pose The {@link Pose} to convert, in FTC standard coordinates
     * @return The converted {@link Pose}, in Pedro coordinate system
     */
    @Override
    public Pose convertToPedro(Pose pose) {
        Pose rotatedPose = pose.rotate(Math.PI / 2, true);
        return rotatedPose.plus(new Pose(72, 72));
    }
}