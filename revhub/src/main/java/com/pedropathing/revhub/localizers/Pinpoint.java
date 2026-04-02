package com.pedropathing.revhub.localizers;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Twist;
import com.pedropathing.geometry.Velocity;
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
