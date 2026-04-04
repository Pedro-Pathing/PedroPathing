package com.pedropathing.revhub.localizers;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.pedropathing.localization.Localizer;

public class Pinpoint implements Localizer {
    @Override
    public Pose getPose() {
        return null;
    }

    @Override
    public void setPose(Pose pose) {

    }

    @Override
    public Twist getTwist() {
        return null;
    }

    @Override
    public Velocity getVelocity() {
        return null;
    }

    @Override
    public void update() {

    }
}
