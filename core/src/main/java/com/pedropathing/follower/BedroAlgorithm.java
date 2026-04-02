package com.pedropathing.follower;

import com.pedropathing.control.Controller;
import com.pedropathing.geometry.Twist;

public class BedroAlgorithm implements Algorithm {
    Controller<?> controller;

    @Override
    public Twist calculate(FollowState state) {
        // Pose pose, Velocity velocity, Twist twist, Path path, PathProgress pathProgress

    }

    public double heading(double current, double target) {
        return controller.calculate(current - target);
    }
}
