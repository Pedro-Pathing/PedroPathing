package com.pedropathing.revhub;

import static com.pedropathing.utils.Angle.normalizeSigned;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class ManualDrive {
    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading) {
        return fieldCentric(powers, currentHeading, 0.0);
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers with an offset heading. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading, double offsetHeading) {
        Vector2D fieldRelative = Vector2D.cartesian(powers.forward(), powers.strafe()).rotate(-(currentHeading + offsetHeading));
        return new DrivePowers(fieldRelative.x(), fieldRelative.y(), powers.turn());
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers. */
    public static DrivePowers fieldCentric(double forward, double lateral, double turn, double currentHeading) {
        return fieldCentric(new DrivePowers(forward, lateral, turn), currentHeading);
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers with an offset heading. */
    public static DrivePowers fieldCentric(double forward, double lateral, double turn, double currentHeading, double offsetHeading) {
        return fieldCentric(new DrivePowers(forward, lateral, turn), currentHeading, offsetHeading);
    }

    public static DrivePowers headingLock(Follower follower, PIDController headingPID, DrivePowers powers, double targetHeading) {
        double headingError = normalizeSigned(targetHeading - follower.pose().heading());
        double power = headingPID.calculate(targetHeading, headingError, follower.twist().omega);
        return new DrivePowers(powers.forward(), powers.strafe(), power);
    }
}
