package com.pedropathing.follower;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.DrivePowers;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;

public class Follower {
    public final Localizer localizer;
    public final Drivetrain drivetrain;
    private Algorithm algorithm;
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
        DrivePowers powers = algorithm.calculate(state);
        drivetrain.drive(powers, algorithm);
    }

    public void follow(Path path) {
        state = new FollowState(path);
    }

    public Pose getPose() {
        return localizer.getPose();
    }

    public Algorithm getAlgorithm() {
        return algorithm;
    }

    public void setAlgorithm(Algorithm algorithm) {
        this.algorithm = algorithm;
    }
}

// Follower follower = new Follower(localizer, drivetrain, new FieldCentricTeleOp(constants));