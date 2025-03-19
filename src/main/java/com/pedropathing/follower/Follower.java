package com.pedropathing.follower;

import static com.pedropathing.follower.FollowerConstants.automaticHoldEnd;
import static com.pedropathing.follower.FollowerConstants.cacheInvalidateSeconds;
import static com.pedropathing.follower.FollowerConstants.drivePIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.drivePIDFSwitch;
import static com.pedropathing.follower.FollowerConstants.forwardZeroPowerAcceleration;
import static com.pedropathing.follower.FollowerConstants.headingPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.headingPIDFSwitch;
import static com.pedropathing.follower.FollowerConstants.lateralZeroPowerAcceleration;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.nominalVoltage;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.secondaryDrivePIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.secondaryHeadingPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.secondaryTranslationalPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.translationalPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.translationalPIDFSwitch;
import static com.pedropathing.follower.FollowerConstants.useSecondaryDrivePID;
import static com.pedropathing.follower.FollowerConstants.useSecondaryHeadingPID;
import static com.pedropathing.follower.FollowerConstants.useSecondaryTranslationalPID;
import static com.pedropathing.follower.FollowerConstants.useVoltageCompensationInAuto;
import static com.pedropathing.follower.FollowerConstants.useVoltageCompensationInTeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.FilteredPIDFController;
import com.pedropathing.util.KalmanFilter;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is the Follower class. It handles the actual following of the paths and all the on-the-fly
 * calculations that are relevant for movement.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class Follower extends FollowerCalculator {
    private HardwareMap hardwareMap;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    /*
     * Voltage Compensation
     * Credit to team 14343 Escape Velocity for the voltage code
     * Credit to team 23511 Seattle Solvers for implementing the voltage code into Follower.java
     */
    private boolean cached = false;

    private VoltageSensor voltageSensor;
    public double voltage = 0;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    /**
     * This creates a new Follower given a HardwareMap.
     * @param hardwareMap HardwareMap required
     */
    public Follower(HardwareMap hardwareMap) {
        super(new PoseUpdater(hardwareMap));
        this.hardwareMap = hardwareMap;
    }

    /**
     * This creates a new Follower given a HardwareMap and a localizer.
     * @param hardwareMap HardwareMap required
     * @param localizer the localizer you wish to use
     */
    public Follower(HardwareMap hardwareMap, Localizer localizer) {
        super(new PoseUpdater(localizer));
        this.hardwareMap = hardwareMap;
    }

    public void initializeHardware() {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageTimer.reset();
    }

    /**
     * This initializes the follower.
     * In this, the DriveVectorScaler and PoseUpdater is instantiated, the drive motors are
     * initialized and their behavior is set, and the variables involved in approximating first and
     * second derivatives for teleop are set.
     */
    @Override
    public void initialize(PoseUpdater poseUpdater) {
        super.initialize(poseUpdater);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageTimer.reset();

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    /**
     * This starts teleop drive control.
     */
    @Override
    public void startTeleopDrive() {
        super.startTeleopDrive();
        if(FollowerConstants.useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    /**
     * This calls an update to the PoseUpdater, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs, which updates the motor powers.
     */
    public void update() {
        super.update();

        double[] drivePowers = getDrivePowers();
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                double voltageNormalized = getVoltageNormalized();

                if (useVoltageCompensationInAuto) {
                    motors.get(i).setPower(drivePowers[i] * voltageNormalized);
                } else {
                    motors.get(i).setPower(drivePowers[i]);
                }
            }
        }
    }

    /**
     * This resets the PIDFs and stops following the current Path.
     */
    public void breakFollowing() {
        super.breakFollowing();

        setMotorsToFloat();
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(0);
        }
    }

    /**
     * @return The last cached voltage measurement.
     */
    public double getVoltage() {
        if (voltageTimer.seconds() > cacheInvalidateSeconds && cacheInvalidateSeconds >= 0) {
            cached = false;
        }

        if (!cached)
            refreshVoltage();

        return voltage;
    }

    /**
     * @return A scalar that normalizes power outputs to the nominal voltage from the current voltage.
     */
    public double getVoltageNormalized() {
        return Math.min(nominalVoltage / getVoltage(), 1);
    }

    /**
     * Overrides the voltage cooldown.
     */
    public void refreshVoltage() {
        cached = true;
        voltage = voltageSensor.getVoltage();
        voltageTimer.reset();
    }
}
