package com.pedropathing.revhub;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Vector2D;

public class ManualDrive {

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading) {
        return fieldCentric(powers, currentHeading, 0.0);
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers with an offset heading. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading, double offset) {
        Vector2D fieldRelative = Vector2D.cartesian(powers.forward(), powers.strafe()).rotate(currentHeading + offset);
        return new DrivePowers(fieldRelative.x(), fieldRelative.y(), powers.turn());
    }
}
