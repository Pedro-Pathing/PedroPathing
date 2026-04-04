package com.pedropathing.follower;

import com.pedropathing.drivetrain.DrivePowers;

public interface Algorithm {
    DrivePowers calculate(FollowState state);
}