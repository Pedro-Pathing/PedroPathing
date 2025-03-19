package com.pedropathing.follower;

import com.pedropathing.pathgen.Vector;

public class FollowerVectors {
    public final Vector correctiveVector;
    public final Vector headingVector;
    public final Vector driveVector;
    public final double currentHeading;

    public FollowerVectors(Vector correctiveVector, Vector headingVector,
                           Vector driveVector, double currentHeading) {
        this.correctiveVector = correctiveVector;
        this.headingVector = headingVector;
        this.driveVector = driveVector;
        this.currentHeading = currentHeading;
    }
}
