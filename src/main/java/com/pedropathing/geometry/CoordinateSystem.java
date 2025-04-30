package com.pedropathing.geometry;

/**
 * <p>A coordinate system, such as FTC standard coordinates or Pedro coordinates</p>
 * <br>
 * FTC standard coordinates are as defined on the
 * <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">FTC Docs</a>.
 *
 * @author BeepBot99
 */
public interface CoordinateSystem {

    /**
     * Converts a {@link Pose} from this coordinate system to FTC standard coordinates
     *
     * @param pose The {@link Pose} to convert, in this coordinate system
     * @return The converted {@link Pose}, in FTC standard coordinates
     */
    Pose convertToFtcStandard(Pose pose);

    /**
     * Converts a {@link Pose} from FTC standard coordinates to this coordinate system
     *
     * @param pose The {@link Pose} to convert, in FTC standard coordinates
     * @return The converted {@link Pose}, in this coordinate system
     */
    Pose convertFromFtcStandard(Pose pose);
}
