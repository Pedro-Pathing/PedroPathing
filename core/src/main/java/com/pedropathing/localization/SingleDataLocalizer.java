package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class SingleDataLocalizer implements Localizer {
    public Pose lastPose;
    public Pose lastVelocity;
    public SingleDataLocalizer(Pose lastPose, Pose lastVelocity) {
        this.lastPose = lastPose;
        this.lastVelocity = lastVelocity;
    }

    @Override
    public Pose getPose() {
        return lastPose;
    }

    @Override
    public Pose getVelocity() {
        return lastVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        throw new RuntimeException("not implemented");
    }

    @Override
    public void setPose(Pose setPose) {
        throw new RuntimeException("not implemented");
    }

    @Override
    public void update() { /*no-op*/ }

    @Override
    public double getTotalHeading() {throw new RuntimeException("not implemented");}
    @Override
    public double getForwardMultiplier() {throw new RuntimeException("not implemented");}
    @Override
    public double getLateralMultiplier() {throw new RuntimeException("not implemented");}
    @Override
    public double getTurningMultiplier() {throw new RuntimeException("not implemented");}
    @Override
    public void resetIMU() {/*no-op*/}
    @Override
    public double getIMUHeading() {throw new RuntimeException("not implemented");}
    @Override
    public boolean isNAN() {return false;}

}
