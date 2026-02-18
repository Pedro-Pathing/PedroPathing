package com.pedropathing.ftc;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

/**
 * An enum that contains the FTC standard coordinate system.
 * This enum implements the {@link CoordinateSystem} interface, which specifies a way to convert to and from FTC standard coordinates.
 *
 * <p>This implementation performs numeric transforms directly on the Pose components
 * to avoid calling {@code Pose} methods that may themselves trigger coordinate conversions.</p>
 *
 * @author BeepBot99
 * @author Baron Henderson
 */
public enum FTCCoordinates implements CoordinateSystem {
    INSTANCE;

    /**
     * Converts a {@link Pose} to this coordinate system from Pedro coordinates
     *
     * @param pose The {@link Pose} to convert, in the Pedro coordinate system
     * @return The converted {@link Pose}, in FTC standard coordinates
     */
    @Override
    public Pose convertFromPedro(Pose pose) {
        Pose newPose = pose.minus(new Pose(72, 72)).rotate(-Math.PI / 2, true);
        return new Pose(newPose.getX(), newPose.getY(), newPose.getHeading(), INSTANCE);
    }

    /**
     * Converts a {@link Pose} to Pedro coordinates from this coordinate system
     *
     * @param pose The {@link Pose} to convert, in FTC standard coordinates
     * @return The converted {@link Pose}, in Pedro coordinate system
     */
    @Override
    public Pose convertToPedro(Pose pose) {
        Pose newPose = new Pose (pose.getX(), pose.getY(), pose.getHeading(), PedroCoordinates.INSTANCE);
        return newPose.rotate(-Math.PI / 2, true).plus(new Pose(72, 72));
    }
}