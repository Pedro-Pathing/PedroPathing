package com.pedropathing.follower;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;
import lombok.Getter;
import lombok.Setter;

public class Follower {
    public final Localizer localizer;
    public final Drivetrain drivetrain;

    @Getter @Setter
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

        if (!isFollowing()) {
            return;
        }

        DrivePowers powers = algorithm.calculate(state);
        drivetrain.drive(powers);
    }

    public void follow(Path path) {
        state = new FollowState(path);
    }

    public Pose getPose() {
        return localizer.getPose();
    }

    public boolean isFollowing() {
        return state.isFollowing();
    }
}