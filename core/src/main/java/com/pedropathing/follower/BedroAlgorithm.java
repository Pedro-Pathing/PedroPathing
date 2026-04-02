package com.pedropathing.follower;

import com.pedropathing.control.Controller;
import com.pedropathing.geometry.Angle;
import com.pedropathing.geometry.Twist;

public class BedroAlgorithm implements Algorithm {
    Controller<?> headingController;

    public BedroAlgorithm(Controller<?> headingController) {
        this.headingController = headingController;
    }

    @Override
    public Twist calculate(FollowState state) {
        // Pose pose, Velocity velocity, Twist twist, Path path
        return new Twist(0, 0, heading(state.getPose().heading, state.getPath().pathProgress.closestPose.heading));
    }

    public double heading(double current, double target) {
        return headingController.calculate(Angle.smallestDifference(current, target) * Angle.turnDirection(current, target));
    }
}
