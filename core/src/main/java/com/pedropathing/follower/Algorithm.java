package com.pedropathing.follower;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Velocity;

public interface Algorithm {
    Drivetrain.Powers calculate(Pose currentPose, Velocity currentVelocity, Path currentPath, PathProgress pathProgress);
}