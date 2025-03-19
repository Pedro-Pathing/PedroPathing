package com.pedropathing.follower;

import com.pedropathing.pathgen.Vector;

/**
 * This is the FollowerVectors class. It holds the vectors used in the
 * Follower when following a path.
 */
public class FollowerVectors {
    public final Vector correctiveVector;
    public final Vector headingVector;
    public final Vector driveVector;
    public final double currentHeading;

    /**
     * This creates a new FollowerVectors object.
     * @param correctiveVector The corrective vector
     * @param headingVector The heading vector
     * @param driveVector The drive vector
     * @param currentHeading The current heading of the robot
     */
    public FollowerVectors(Vector correctiveVector, Vector headingVector,
                           Vector driveVector, double currentHeading) {
        this.correctiveVector = correctiveVector;
        this.headingVector = headingVector;
        this.driveVector = driveVector;
        this.currentHeading = currentHeading;
    }
}
