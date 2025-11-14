package com.pedropathing.ftc.drivetrains;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


/**
 * Swerve drivetrain implementation
 * @author Kabir Goyal - 365 MOE
 */
public class Swerve extends CustomDrivetrain {
    SwervePod[] pods;
    SwervePod frontLeftPod;
    SwervePod frontRightPod;
    SwervePod backLeftPod;
    SwervePod backRightPod;


    /**
     * Constructs a SwerveDrive with the specified pods.
     * @param swerveConstants Fill out your SwerveConstants class!
     */
    public Swerve(HardwareMap hardwareMap, SwerveConstants swerveConstants)  {
        pods = new SwervePod[4];

        frontLeftPod = new SwervePod(hardwareMap, swerveConstants.leftFrontServoName, swerveConstants.leftFrontEncoderName,
                swerveConstants.leftFrontMotorName, swerveConstants.leftFrontTurnPID,
                swerveConstants.leftFrontMotorDirection, swerveConstants.leftFrontServoReversed,
                swerveConstants.leftFrontPodAngleOffsetDeg, swerveConstants.leftFrontPodXYOffsets);

        frontRightPod = new SwervePod(hardwareMap, swerveConstants.rightFrontServoName, swerveConstants.rightFrontEncoderName,
                swerveConstants.rightFrontMotorName, swerveConstants.rightFrontTurnPID,
                swerveConstants.rightFrontMotorDirection, swerveConstants.rightFrontServoReversed,
                swerveConstants.rightFrontPodAngleOffsetDeg, swerveConstants.rightFrontPodXYOffsets);

        backLeftPod = new SwervePod(hardwareMap, swerveConstants.leftRearServoName, swerveConstants.leftRearEncoderName,
                swerveConstants.leftRearMotorName, swerveConstants.leftRearTurnPID,
                swerveConstants.leftRearMotorDirection, swerveConstants.leftRearServoReversed,
                swerveConstants.leftRearPodAngleOffsetDeg, swerveConstants.leftRearPodXYOffsets);

        backRightPod = new SwervePod(hardwareMap, swerveConstants.rightRearServoName, swerveConstants.rightRearEncoderName,
                swerveConstants.rightRearMotorName, swerveConstants.rightRearTurnPID,
                swerveConstants.rightRearMotorDirection, swerveConstants.rightRearServoReversed,
                swerveConstants.rightRearPodAngleOffsetDeg, swerveConstants.rightRearPodXYOffsets);

        pods[0] = frontLeftPod;
        pods[1] = frontRightPod;
        pods[2] = backLeftPod;
        pods[3] = backRightPod;
    }

    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {

        //stores forward and strafe values as the translation vector with max magnitude of 1
        Vector rawTrans = new Vector(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean ignoreTrans = rawTrans.getMagnitude() < 0.05;
        boolean ignoreRotation = Math.abs(rotation) < 0.05;

        double rotationScalar = (ignoreRotation) ? 0 : rotation;

        Vector[] podVectors = new Vector[pods.length];

        for (int i = 0; i < pods.length; i++) {
            SwervePod pod = pods[i];

            Vector translationVector = ignoreTrans ? new Vector(0, 0) : rawTrans;

            //we want -rotationScalar because after the vector rotation, the vectors will be pointing counterclockwise,
            // which is what we want when rotation < 0 (joystick to left)
            Vector rotationVector = new Vector(-rotationScalar, Math.atan2(pod.getYOffset(), pod.getXOffset()));

            //this gets the perpendicular vector for the wheel
            rotationVector.rotateVector(Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
        }

        //finding if any vector has magnitude > 1
        double maxMagnitude = 1;
        for (int i = 0; i < podVectors.length; i++) {
            maxMagnitude = Math.max(maxMagnitude, podVectors[i].getMagnitude());
        }

        for (int podNum = 0; podNum < pods.length; podNum++) {
            Vector finalVector = podVectors[podNum].times(1.0 / maxMagnitude); //Normalizing if necessary while preserving relative sizes
            //2*Pi-theta because servos have positive clockwise rotation, while our angles are counterclockwise
            pods[podNum].move(2 * Math.PI - finalVector.getTheta(), finalVector.getMagnitude(),
                    ignoreTrans && ignoreRotation);
        }
    }

    @Override
    public void updateConstants() {
        //TODO
    }

    @Override
    public void breakFollowing() {
        //TODO
    }

    @Override
    public void startTeleopDrive() {
        //TODO
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {

        //TODO

    }

    @Override
    public double xVelocity() {
        return 0;
    }

    @Override
    public double yVelocity() {
        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {
        //TODO
    }

    @Override
    public void setYVelocity(double yMovement) {
        //TODO
    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public String debugString() {
        return "";
    }
}
