package com.pedropathing.ftc.drivetrains;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.List;

public class Tank extends CustomDrivetrain {
    public TankConstants constants;
    private final VoltageSensor voltageSensor;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;
    private final List<DcMotorEx> motors;

    public Tank(HardwareMap hardwareMap, TankConstants constants) {
        this.constants = constants;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        leftFront = hardwareMap.get(DcMotorEx.class, constants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, constants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, constants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, constants.rightFrontMotorName);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        setMotorsToFloat();
        breakFollowing();
    }

    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {
        double leftPower = forward + rotation;
        double rightPower = forward - rotation;

        if (voltageCompensation) {
            double normalizedVoltage = getVoltageNormalized();
            leftPower *= normalizedVoltage;
            rightPower *= normalizedVoltage;
        }

        double maxPower = Math.max(leftPower, rightPower);
        if (maxPower > constants.maxPower) {
            leftPower = leftPower / maxPower * maxPowerScaling;
            rightPower = rightPower / maxPower * maxPowerScaling;
        }

        double[] drivePowers = new double[] {leftPower, leftPower, rightPower, rightPower};
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > constants.motorCachingThreshold) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (headingPower.getMagnitude() > maxPowerScaling) {
            headingPower.setMagnitude(maxPowerScaling);
            return new double[] {
                    0,
                    0,
                    headingPower.getMagnitude()
            };
        }

        double targetHeading = correctivePower.plus(pathingPower).getTheta();
        double multiplier = Math.cos(targetHeading - robotHeading);
        correctivePower.setComponents(multiplier * correctivePower.getMagnitude(), 0);
        pathingPower.setComponents(multiplier * pathingPower.getMagnitude(),0);

        if (correctivePower.getMagnitude() >= maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);

        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        if (scaleDown(headingPower, correctivePower, true)) {
            correctivePower = scaledVector(headingPower, correctivePower, true);
            return new double[] {
                    correctivePower.getXComponent(),
                    0,
                    headingPower.dot(new Vector(1, robotHeading))
            };
        } else {
            Vector combinedStatic = correctivePower.plus(headingPower);
            if (scaleDown(combinedStatic, pathingPower, false)) {
                pathingPower = scaledVector(combinedStatic, pathingPower, false);
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        0,
                        headingPower.dot(new Vector(1, robotHeading))
                };
            } else {
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        0,
                        headingPower.dot(new Vector(1, robotHeading))
                };
            }
        }
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);
        Vector positionVector = new Vector();
        positionVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);
        lastPositionVector = positionVector;
        lastRotationalVector = new Vector(calculatedDrive[2], robotHeading);
        arcadeDrive(positionVector.getXComponent(), 0, calculatedDrive[2]);
    }

    @Override
    public void updateConstants() {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
    }

    @Override
    public void startTeleopDrive() {
        startTeleopDrive(constants.useBrakeModeInTeleOp);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode)
            setMotorsToBrake();
        else
            setMotorsToFloat();
    }

    @Override
    public double xVelocity() {
        return constants.xVelocity;
    }

    @Override
    public double yVelocity() {
        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {
        constants.xVelocity = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {}

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return "";
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

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * constants.staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * constants.staticFrictionCoefficient));
    }
}
