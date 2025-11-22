package com.pedropathing;

import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;

public abstract class Drivetrain {
    /**
     * These are the movement vectors for the drivetrain, essentially the directions of the forces created by each wheel, such that x-components are scaled by the x velocity and y-components are scaled by the y velocity.
     */
    protected Vector[] vectors;

    /**
     * This is the maximum power scaling for the drivetrain. This is used to limit the maximum
     * power that can be applied to the motors, which is useful for preventing damage to the
     * drivetrain or for controlling the speed of the robot.
     */
    protected double maxPowerScaling;

    /**
     * This is used to determine whether the drivetrain should use voltage compensation or not.
     */
    protected boolean voltageCompensation;

    /**
     * This is the nominal voltage for the drivetrain. This is used for voltage compensation.
     * It is set to 12.0V by default, which is the nominal voltage for most FTC robots.
     */
    protected double nominalVoltage;

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array.
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heading, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public abstract double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    /**
     * This sets the maximum power scaling for the drivetrain. This is used to limit the maximum
     * power that can be applied to the motors, which is useful for preventing damage to the
     * drivetrain or for controlling the speed of the robot.
     *
     * @param maxPowerScaling this is a double between 0 and 1 inclusive that represents the maximum
     *                        power scaling factor.
     */
    public void setMaxPowerScaling(double maxPowerScaling) {
        this.maxPowerScaling = MathFunctions.clamp(maxPowerScaling, 0, 1);
    }

    /**
     * This gets the maximum power scaling for the drivetrain. This is used to limit the maximum
     * power that can be applied to the motors, which is useful for preventing damage to the
     * drivetrain or for controlling the speed of the robot.
     *
     * @return this returns a double between 0 and 1 inclusive that represents the maximum power
     *         scaling factor.
     */
    public double getMaxPowerScaling() {
        return maxPowerScaling;
    }

    /**
     * This updates the constants used by the drivetrain.
     */
    public abstract void updateConstants();

    /**
     * This is used to break the drivetrain's following. This is useful for stopping the robot from following a Path or PathChain.
     */
    public abstract void breakFollowing();

    /**
     * This runs the drivetrain with the specified drive powers. This is used to set the power of the motors directly.
     *
     * @param drivePowers this is an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public abstract void runDrive(double[] drivePowers);

    /**
     * This gets the drive powers and runs them immediately.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heading, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     */
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    /**
     * This starts the TeleOp drive mode. This is used to set the drivetrain into TeleOp mode, where
     * it can be controlled by the driver.
     */
    public abstract void startTeleopDrive();

    /**
     * This starts the TeleOp drive mode with a specified brake mode. This is used to set the drivetrain
     * into TeleOp mode, where it can be controlled by the driver, and allows for setting the brake mode.
     *
     * @param brakeMode this is a boolean that specifies whether the drivetrain should use brake mode or not.
     */
    public abstract void startTeleopDrive(boolean brakeMode);

    /**
     * This gets the current x velocity of the drivetrain.
     * @return this returns the x velocity of the drivetrain.
     */
    public abstract double xVelocity();

    /**
     * This gets the current y velocity of the drivetrain.
     * @return this returns the y velocity of the drivetrain.
     */
    public abstract double yVelocity();

    /**
     * This sets the x velocity of the drivetrain.
     * @param xMovement this is the x velocity to set.
     */
    public abstract void setXVelocity(double xMovement);

    /**
     * This sets the y velocity of the drivetrain.
     * @param yMovement this is the y velocity to set.
     */
    public abstract void setYVelocity(double yMovement);

    /**
     * This sets whether the drivetrain should use voltage compensation or not.
     * @param use this is a boolean that specifies whether the drivetrain should use voltage compensation or not.
     */
    public void useVoltageCompensation(boolean use) {
        this.voltageCompensation = use;
    }

    /**
     * This gets whether the drivetrain is using voltage compensation or not.
     * @return this returns a boolean that specifies whether the drivetrain is using voltage compensation or not.
     */
    public boolean isVoltageCompensation() {
        return voltageCompensation;
    }

    /**
     * This gets the nominal voltage for the drivetrain.
     * @return this returns the nominal voltage for the drivetrain.
     */
    public double getNominalVoltage() {
        return nominalVoltage;
    }

    /**
     * This sets the nominal voltage for the drivetrain. This is used for voltage compensation.
     * @param set this is the nominal voltage to set.
     */
    public void setNominalVoltage(double set) {
        this.nominalVoltage = set;
    }

    /**
     * This is used to get the voltage of the drivetrain. This is useful for debugging purposes.
     * It should be called periodically to get the current voltage of the drivetrain.
     */
    public abstract double getVoltage();

    /**
     * This is used to get a debug string for the drivetrain. This is useful for debugging purposes.
     *
     * @return this returns a String that contains the debug information for the drivetrain.
     */
    public abstract String debugString();

}
