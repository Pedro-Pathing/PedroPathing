package com.pedropathing.follower;

import com.pedropathing.geometry.DriveCommand;

public interface Algorithm {
    DriveCommand calculate(FollowState state);
}