package com.pedropathing.ftc.drivetrains;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

/**
 * This is the MecanumEx class, a version of Mecanum using CustomDrivetrain
 * @author Kabir Goyal
 * @version 1.0, 4/30/2025
 */
public class MecanumEx extends CustomDrivetrain {
    public MecanumConstants constants;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;
    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;

    /**
     * This creates a new Mecanum, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     *
     * @param hardwareMap      this is the HardwareMap object that contains the motors and other hardware
     * @param mecanumConstants this is the MecanumConstants object that contains the names of the motors and directions etc.
     */
    public MecanumEx(HardwareMap hardwareMap, MecanumConstants mecanumConstants) {
        constants = mecanumConstants;

        this.maxPowerScaling = mecanumConstants.maxPower;
        this.motorCachingThreshold = mecanumConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = mecanumConstants.useBrakeModeInTeleOp;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.get(DcMotorEx.class, mecanumConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, mecanumConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, mecanumConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, mecanumConstants.rightFrontMotorName);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();

        Vector copiedFrontLeftVector = mecanumConstants.frontLeftVector.normalize();
        vectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};
    }

    /**
     * This method takes in forward, strafe, and rotation values and applies them to
     * the drivetrain.
     *
     * @param forward the forward power value, which would typically be
     *                -gamepad1.left_stick_y in a normal arcade drive setup
     * @param strafe the strafe power value, which would typically be
     *               -gamepad1.left_stick_x in a normal arcade drive setup
     *               because pedro treats left as positive
     * @param rotation the rotation power value, which would typically be
     *                 -gamepad1.right_stick_x in a normal arcade drive setup
     *                 because CCW is positive
     */
    public void arcadeDrive(double forward, double strafe, double rotation) {
        double[] wheelPowers = new double[4];

        wheelPowers[0] = forward + strafe - rotation; //leftFront
        wheelPowers[1] = forward - strafe + rotation; //rightFront
        wheelPowers[2] = forward - strafe - rotation; //leftRear
        wheelPowers[3] = forward + strafe + rotation; //rightRear

        double denom = 1;
        for (double power : wheelPowers) {
           denom = Math.max(denom, Math.abs(power));
        }

        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = wheelPowers[i] / denom;
        }

        leftFront.setPower(wheelPowers[0]);
        rightFront.setPower(wheelPowers[1]);
        leftRear.setPower(wheelPowers[2]);
        rightRear.setPower(wheelPowers[3]);
    }

    @Override
    public void updateConstants() {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
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

    @Override
    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        super.runDrive(correctivePower, headingPower, pathingPower, robotHeading);
    }

    @Override
    public double xVelocity() {
        return constants.xVelocity;
    }

    @Override
    public double yVelocity() {
        return constants.yVelocity;
    }

    @Override
    public void setXVelocity(double xMovement) { constants.setXVelocity(xMovement); }
    @Override
    public void setYVelocity(double yMovement) { constants.setYVelocity(yMovement); }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public String debugString() {
        return "Mecanum{" +
                " leftFront=" + leftFront +
                ", leftRear=" + leftRear +
                ", rightFront=" + rightFront +
                ", rightRear=" + rightRear +
                ", motors=" + motors +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}';
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }
}
