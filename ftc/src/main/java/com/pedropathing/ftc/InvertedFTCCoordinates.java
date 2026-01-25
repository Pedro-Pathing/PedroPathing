package com.pedropathing.ftc;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

/**
 * An enum that contains the Inverted FTC standard coordinate system (for DECODE game).
 * This enum implements the {@link CoordinateSystem} interface, which specifies a way to convert to and from Inverted FTC standard coordinates.
 *
 * @author BeepBot99
 * @author Baron Henderson
 */
public enum InvertedFTCCoordinates implements CoordinateSystem {
    INSTANCE;

    /**
     * Converts a {@link Pose} to this coordinate system from Pedro coordinates
     *
     * @param pose The {@link Pose} to convert, in the Pedro coordinate system
     * @return The converted {@link Pose}, in Inverted FTC standard coordinates
     */
    @Override
    public Pose convertFromPedro(Pose pose) {
        // Center the pose (subtract offset without using minus to avoid coordinate conversion issues)
        Pose centered = new Pose(pose.getX() - 72, pose.getY() - 72, pose.getHeading());
        Pose rotated = centered.rotate(Math.PI / 2, true);
        // Return with InvertedFTC coordinate system
        return new Pose(rotated.getX(), rotated.getY(), rotated.getHeading(), InvertedFTCCoordinates.INSTANCE);
    }

    /**
     * Converts a {@link Pose} to Pedro coordinates from this coordinate system
     *
     * @param pose The {@link Pose} to convert, in Inverted FTC standard coordinates
     * @return The converted {@link Pose}, in Pedro coordinate system
     */
    @Override
    public Pose convertToPedro(Pose pose) {
        // Rotate first (inverse rotation of convertFromPedro)
        Pose rotated = pose.rotate(-Math.PI / 2, true);
        // Add offset and return with Pedro coordinate system
        return new Pose(rotated.getX() + 72, rotated.getY() + 72, rotated.getHeading(), PedroCoordinates.INSTANCE);
    }
}