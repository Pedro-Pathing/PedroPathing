package com.pedropathing.drivetrain;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;
import static com.pedropathing.math.MathFunctions.normalizeAngle;

import com.pedropathing.math.Vector;

/**
 * This is the CustomDrivetrain class. This is an abstract class that extends the Drivetrain class.
 * It is intended to be used as a base class for custom drivetrain implementations.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public abstract class CustomDrivetrain extends Drivetrain {
    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;

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

    private boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    private Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus? Math.min(findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling)) :
                findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);
        Vector translationalVector = new Vector();
        translationalVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);
        lastPathingPower = pathingPower;
        lastCorrectivePower = correctivePower;

        lastTranslationalVector = translationalVector; //before rotation
        lastHeadingPower = headingPower;
        lastHeading = robotHeading;

        translationalVector.rotateVector(-robotHeading); // this should make it field centric when field centric is desired and robot centric otherwise
        arcadeDrive(translationalVector.getXComponent(), translationalVector.getYComponent(), calculatedDrive[2]);
    }

    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {}
}
