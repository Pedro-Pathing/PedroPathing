/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.FollowState;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.compiled.PathProgress;
import lombok.experimental.Accessors;
import lombok.experimental.Delegate;

public class Follower {
    @Delegate
    public final Localizer localizer;

    public final Drivetrain drivetrain;
    private @Accessors(fluent = false) Algorithm algorithm;
    private PathProgress pathProgress;

    public Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm) {
        this.localizer = localizer;
        this.drivetrain = drivetrain;
        this.algorithm = algorithm;
    }

    public void update() {
        localizer.update();
        if (pathProgress != null) pathProgress.update(localizer.getPose());

        FollowState state =
                new FollowState(localizer.getPose(), localizer.getVelocity(), localizer.getTwist(), pathProgress);
        DrivePowers powers = algorithm.calculate(state);
        drivetrain.drive(powers, algorithm);
    }

    public void follow(Path path) {
        pathProgress = new PathProgress(path);
    }
}
