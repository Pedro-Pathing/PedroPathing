package com.pedropathing.follower;

import com.pedropathing.geometry.DrivePowers;

public interface Algorithm {
    DrivePowers calculate(FollowState state);
}