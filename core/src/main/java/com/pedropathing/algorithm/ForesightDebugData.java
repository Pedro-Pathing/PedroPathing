package com.pedropathing.algorithm;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.DataInfo;

public class ForesightDebugData implements DataInfo {
    public final Pose pose;
    public final Velocity velocity;
    public final Twist twist;
    public final double translationalError;
    public final double headingError;
    public final double tangentialVelocity;
    public final double closestT;
    public final double projectedClosestT;
    public final double remainingDistance;
    public final double completion;

    private final Vector2D normalFeedforwardVector;
    private final double headingFeedforward;
    private final Vector2D translationalVector;
    private final Vector2D driveVector;
    private final double headingPower;

    public ForesightDebugData(Pose pose, Velocity velocity, Twist twist, double translationalError, double headingError,
                              double tangentialVelocity, double closestT, double projectedClosestT, double remainingDistance,
                              double completion, Vector2D normalFeedforwardVector, double headingFeedforward, Vector2D translationalVector, Vector2D driveVector, double headingPower) {
        this.pose = pose;
        this.velocity = velocity;
        this.twist = twist;
        this.translationalError = translationalError;
        this.headingError = headingError;
        this.tangentialVelocity = tangentialVelocity;
        this.closestT = closestT;
        this.projectedClosestT = projectedClosestT;
        this.remainingDistance = remainingDistance;
        this.completion = completion;
        this.normalFeedforwardVector = normalFeedforwardVector;
        this.headingFeedforward = headingFeedforward;
        this.translationalVector = translationalVector;
        this.driveVector = driveVector;
        this.headingPower = headingPower;
    }

    @Override
    public String toString() {
        return "ForesightDebugData {" +
                "\n Pose: " + pose +
                "\n Velocity: " + velocity +
                "\n Twist: " + twist +
                "\n Translational Error: " + translationalError +
                "\n Heading Error: " + headingError +
                "\n Tangential Velocity: " + tangentialVelocity +
                "\n Closest T: " + closestT +
                "\n Projected Closest T: " + projectedClosestT +
                "\n Remaining Distance: " + remainingDistance +
                "\n Completion: " + completion +
                "\n Normal Feedforward: " + normalFeedforwardVector +
                "\n Heading Feedforward: " + headingFeedforward +
                "\n Translational Vector: " + translationalVector +
                "\n Drive Vector: " + driveVector +
                "\n Heading Power: " + headingPower +
                "\n}";
    }
}
