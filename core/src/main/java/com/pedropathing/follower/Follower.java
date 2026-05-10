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
import com.pedropathing.paths.api.Path;
import lombok.AllArgsConstructor;
import lombok.experimental.Accessors;
import lombok.experimental.Delegate;

@AllArgsConstructor
public class Follower {
    @Delegate
    public final Localizer localizer;

    public final Drivetrain drivetrain;
    private @Accessors(fluent = false) Algorithm algorithm;

    public void update() {
        localizer.update();
        FollowState state = new FollowState(localizer.getPose(), localizer.getVelocity(), localizer.getTwist());
        DrivePowers powers = algorithm.calculate(state);
        drivetrain.drive(powers, algorithm);
    }

    public void follow(Path path) {
        state = new FollowState(path);
    }
}
