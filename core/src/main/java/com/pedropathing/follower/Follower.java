package com.pedropathing.follower;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathTracker;
import lombok.Getter;
import lombok.Setter;

public class Follower {
    @Getter
    public final Localizer localizer;
    @Getter
    public final Drivetrain drivetrain;

    @Getter @Setter
    private Algorithm algorithm;
    @Getter
    private FollowState state = null;
    private PathTracker pathTracker = null;
    private double currentNanoTime;
    @Getter
    private boolean manual = false;

    public Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm) {
        this.localizer = localizer;
        this.algorithm = algorithm;
        this.drivetrain = drivetrain;
    }

    public void update() {
        localizer.update();

        if (manual) return;

        double previousNanoTime = currentNanoTime;
        currentNanoTime = System.nanoTime();
        state = new FollowState(localizer.motionState(), pathTracker, currentNanoTime - previousNanoTime);

        if (pathTracker == null || pathTracker.empty()) {
            pathTracker = null;
            DrivePowers powers = algorithm.hold(state.motionState().pose(), state);
            drivetrain.drive(powers);
        }

        DrivePowers powers = algorithm.calculate(state);
        drivetrain.drive(powers);
    }

    public void follow(Path path) {
        manual = false;
        pathTracker = new PathTracker(path);
    }

    // TODO: hold(pose)

    public void manual(double forward, double lateral, double heading) {
        stop();
        drivetrain.manual(new DrivePowers(forward, lateral, heading));
    }

    public void stop() {
        manual = true;
        pathTracker = null;
    }

    public Pose pose() {
        return localizer.pose();
    }

    public boolean isBusy() {
        if (manual || pathTracker == null) return false;
        return pathTracker.isBusy();
    }
}