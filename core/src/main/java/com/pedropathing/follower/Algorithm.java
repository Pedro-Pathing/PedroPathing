package com.pedropathing.follower;

import com.pedropathing.geometry.Twist;

public interface Algorithm {
    Twist calculate(FollowState state);
}