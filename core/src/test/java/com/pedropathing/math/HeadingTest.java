package com.pedropathing.math;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import java.util.Arrays;

public class HeadingTest {
    public static void main(String[] args) {
        Foresight foresight = new Foresight(new ForesightConfig(c -> {}));
        Pose current = new Pose(0, 0, 0);
        MotionState state = MotionState.ofVelocity(current, new Velocity(2, 5, Math.PI / 4));
        Pose target = new Pose(15, 20, Math.PI/2);
        double headingError = foresight.headingError(current.heading(), target.heading());
        double headingPower = foresight.headingPower(headingError, state);
        System.out.println(headingError);
        System.out.println(headingPower);

        DrivePowers powers = foresight.getDrivePowers(Vector2D.zero(), state, headingPower);
        System.out.println(powers);

        System.out.println(Arrays.toString(computeWheelPowers(powers)));
    }

    public static double[] computeWheelPowers(DrivePowers powers) {
        double[] wheelPowers = new double[4];
        double forward = powers.forward();
        double strafe = powers.strafe();
        double turn = powers.turn();

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        double scale = 1 / max;

        final int FL = 0;
        final int FR = 1;
        final int BL = 2;
        final int BR = 3;

        wheelPowers[FL] = fl * scale;
        wheelPowers[FR] = fr * scale;
        wheelPowers[BL] = bl * scale;
        wheelPowers[BR] = br * scale;

        return wheelPowers;
    }
}
