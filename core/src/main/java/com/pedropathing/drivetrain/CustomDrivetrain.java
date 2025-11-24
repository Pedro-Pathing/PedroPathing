package com.pedropathing.drivetrain;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.math.Vector;

/**
 * This is the CustomDrivetrain class. This is an abstract class that extends the Drivetrain class.
 * It is intended to be used as a base class for custom drivetrain implementations.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public abstract class CustomDrivetrain extends Drivetrain {
    protected Vector lastPositionVector;
    protected Vector lastRotationalVector;
    protected boolean useFieldCentric = false;

    /**
     * This method takes in forward, strafe, and rotation values and applies them to the drivetrain.
     * Intended to work exactly like an arcade drive would in a typical TeleOp, this method can be a copy pasted from
     * a robot-centric arcade drive implementation.
     *
     * @param forward the forward power value, which would typically be -gamepad1.left_stick_y in a normal arcade drive setup.
     * @param strafe the strafe power value, which would typically be gamepad1.left_stick_x in a normal arcade drive setup.
     * @param rotation the rotation power value, which would typically be gamepad1.right_stick_x in a normal arcade drive setup.
     */
    public abstract void arcadeDrive(double forward, double strafe, double rotation);

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() >= maxPowerScaling) {
            correctivePower.setMagnitude(maxPowerScaling);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    0
            };
        }

        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        if (scaleDown(correctivePower, headingPower, true)) {
            headingPower = scaledVector(correctivePower, headingPower, true);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    headingPower.dot(new Vector(1, robotHeading))
            };
        } else {
            Vector combinedStatic = correctivePower.plus(headingPower);
            if (scaleDown(combinedStatic, pathingPower, false)) {
                pathingPower = scaledVector(combinedStatic, pathingPower, false);
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            } else {
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            }
        }
    }

    protected boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    protected Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus? Math.min(findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling)) :
                findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower.copy(), headingPower.copy(), pathingPower.copy(), robotHeading);
        Vector positionVector = new Vector();
        positionVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);
        lastPositionVector = positionVector;
        lastRotationalVector = new Vector(calculatedDrive[2], robotHeading);
        if (!useFieldCentric)
            positionVector.rotateVector(-robotHeading);
        arcadeDrive(positionVector.getXComponent(), positionVector.getYComponent(), calculatedDrive[2]);
    }

    public Vector getLastRotationalVector() {
        return lastRotationalVector;
    }

    public Vector getLastPositionVector() {
        return lastPositionVector;
    }

    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {}
}