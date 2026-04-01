package com.pedropathing.follower;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

public class Follower {
    Localizer localizer;
    Algorithm algorithm;
    Drivetrain drivetrain;
    FollowState state;

    public void update() {
        localizer.update();
        state = new FollowState(localizer.getPose(), localizer.getVelocity(), localizer.getTwist(), state.path, state.pathProgress);
        Drivetrain.Powers powers = algorithm.calculate(state);
        drivetrain.drive(powers);
    }

    public void follow(Path path) {
        state.updatePath(path);
        // update progress
    }

    public Pose getPose() {
        return state.pose;
    }
}
