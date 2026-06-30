package com.pedropathing.revhub;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ManualDrive {
    private boolean fieldCentric = false;
    private double offsetHeading = 0;
    private boolean headingLock = false;

    private final Follower follower;

    public ManualDrive(Follower follower) {
        this.follower = follower;
    }

    //TODO: Write robot-centric and field-centric code, offset vector, heading lock, etc
    public DrivePowers computeDrive(Gamepad gamepad) {
        return null;
    }
}
