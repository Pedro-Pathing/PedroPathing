package com.pedropathing.drivetrain;

import com.pedropathing.follower.Algorithm;
import com.pedropathing.geometry.Twist;

public interface Drivetrain {
    void drive(Twist powers, Algorithm algorithm); // TODO: algorithm is so that in the specific dts we can change behavior of the drivetrain, e.g. for mecanum we can have a field-centric algorithm and a robot-centric algorithm or brakeModeInTeleop
    void stop();
}
