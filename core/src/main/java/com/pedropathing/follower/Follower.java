package com.pedropathing.follower;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Twist;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;

public class Follower {
    private final Localizer localizer;
    private Algorithm algorithm;
    private final Drivetrain drivetrain;
    private FollowState state;

    public Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm) {
        this.localizer = localizer;
        this.algorithm = algorithm;
        this.drivetrain = drivetrain;
        this.state = new FollowState(null);
    }

    public void update() {
        localizer.update();
        state.update(localizer.getPose(), localizer.getVelocity(), localizer.getTwist());
        Twist powers = algorithm.calculate(state);
        drivetrain.drive(powers, algorithm);
    }

    public void follow(Path path) {
        state = new FollowState(path);
    }

    public Pose getPose() {
        return localizer.getPose();
    }
}
