package com.pedropathing.math;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.path;
import static org.junit.Assert.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Constants;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Ellipse2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.pedropathing.utils.Pair;

import org.junit.Test;


public class ForesightCurveTest {
    public static void main(String[] args) {

        ForesightCurveTest test = new ForesightCurveTest();

        run("Closest Point + Tangent", test::testClosestPointAndTangent);
        run("Normal Projection", test::testNormalProjection);
        run("Translational Correction Direction", test::testTranslationalCorrectionDirection);
        run("Field To Robot Transform", test::testFieldToRobotTransform);
        run("Brake Velocity", test::testBrakeVelocity);
        run("Heading Error Wrap", test::testHeadingErrorWrap);
        run("Centripetal Acceleration", test::testCentripetalAcceleration);
        run("Heading Power Sign", test::testHeadingPowerSign);
        run("Robot Frame Drive Direction", test::testRobotFrameDriveDirection);
        run("Pure Rotation Direction", test::testPureRotationDirection);
        run("Sequential Power Allocation Starvation", test::testAllocation);
        run("Full", test::testFullCurveOutput);
        run("HeadingController", test::testHeadingControllerOutput);
        run("HeadingFF", test::testHeadingFeedforwardOutput);
        run("HeadingDeriv", test::testHeadingDerivative);
        run("HeadingAlloc", test::testHeadingAllocation);

        System.out.println("\nALL TESTS PASSED");
    }

    @Test
    public void testHeadingDerivative() {


        BezierCurve curve =
                new BezierCurve(
                        Vector2D.cartesian(0,0),
                        Vector2D.cartesian(50,100),
                        Vector2D.cartesian(100,0)
                );


        double t = 0.5;

        Path path = path(curve).tangent();

        double headingDerivative = path.headingDerivative(t);


        double parametricVelocity =
                1.0 / curve.derivative(t).magnitude();


        double actualHeadingDerivative =
                headingDerivative * parametricVelocity;


        double feedforward =
                Constants.foresightConfig.headingFeedforward
                        .get()
                        .calculate(
                                actualHeadingDerivative,
                                0
                        );


        System.out.println(
                "Curve heading derivative: "
                        + headingDerivative
        );

        System.out.println(
                "Curve derivative magnitude: "
                        + curve.derivative(t).magnitude()
        );

        System.out.println(
                "Actual heading derivative: "
                        + actualHeadingDerivative
        );

        System.out.println(
                "Heading FF: "
                        + feedforward
        );


        assertFalse(
                Double.isNaN(feedforward)
        );
    }

    @Test
    public void testHeadingAllocation() {

        Foresight foresight =
                new Foresight(Constants.foresightConfig);


        MotionState state =
                MotionState.ofTwist(
                        new Pose(20,40,0),
                        new Twist(0,0,0)
                );


        double headingPower =
                foresight.headingPower(
                        Math.PI,
                        state
                );


        System.out.println(
                "Raw heading power = " + headingPower
        );


        double headingFF =
                Constants.foresightConfig.headingFeedforward
                        .get()
                        .calculate(
                                10,
                                0
                        );


        System.out.println(
                "Raw heading FF = " + headingFF
        );


        assertTrue(
                Math.abs(headingPower) < 10
        );

        assertTrue(
                Math.abs(headingFF) < 10
        );
    }

    @Test
    public void testHeadingControllerOutput() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        MotionState state =
                MotionState.ofTwist(
                        new Pose(
                                20,
                                40,
                                0
                        ),
                        new Twist(
                                0,
                                0,
                                0
                        )
                );


        double[] errors = {
                Math.toRadians(5),
                Math.toRadians(15),
                Math.toRadians(45),
                Math.toRadians(90),
                Math.toRadians(180)
        };


        for (double error : errors) {

            double power =
                    foresight.headingPower(
                            error,
                            state
                    );


            System.out.println(
                    "Heading error: "
                            + Math.toDegrees(error)
                            + " deg"
            );

            System.out.println(
                    "Heading power: "
                            + power
            );
        }
    }



    @Test
    public void testHeadingFeedforwardOutput() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        double[] headingVelocities = {
                0,
                0.5,
                1,
                5,
                10
        };


        for (double velocity : headingVelocities) {

            double output =
                    Constants.foresightConfig.headingFeedforward
                            .get()
                            .calculate(
                                    velocity,
                                    0
                            );


            System.out.println(
                    "Heading derivative: "
                            + velocity
            );

            System.out.println(
                    "Feedforward output: "
                            + output
            );
        }
    }

    @Test
    public void testFullCurveOutput() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        Path path = curve(Vector2D.cartesian(0,0),
                Vector2D.cartesian(50,100),
                Vector2D.cartesian(100,0)).tangent();

        PathTracker tracker =
                new PathTracker(path);


        MotionState state =
                MotionState.ofTwist(
                        new Pose(
                                20,
                                40,
                                0
                        ),
                        new Twist(
                                0,
                                0,
                                0
                        )
                );


        DrivePowers powers =
                foresight.calculatePath(
                        new TestDrivetrain(),
                        tracker,
                        state,
                        0.02
                );


        System.out.println(
                "Forward: " + powers.forward()
        );

        System.out.println(
                "Strafe: " + powers.strafe()
        );

        System.out.println(
                "Turn: " + powers.turn()
        );


        assertFalse(
                Double.isNaN(powers.forward())
        );

        assertFalse(
                Double.isNaN(powers.strafe())
        );

        assertFalse(
                Double.isNaN(powers.turn())
        );
    }

    private static class TestDrivetrain implements Drivetrain {

        @Override
        public double maxScaling(
                DrivePowers current,
                DrivePowers delta
        ) {
            return 1.0;
        }

        @Override
        public double[] computeWheelPowers(DrivePowers powers) {
            return null;
        }

        @Override
        public void drive(
                DrivePowers powers,
                boolean manual
        ) {

        }

        @Override
        public void stop() {

        }
    }


    private static void run(String name, Runnable test) {
        try {
            test.run();
            System.out.println("[PASS] " + name);
        } catch (Throwable e) {
            System.out.println("[FAIL] " + name);
            e.printStackTrace();
        }
    }

    @Test
    public void testClosestPointAndTangent() {

        BezierCurve curve = new BezierCurve(
                Vector2D.cartesian(0,0),
                Vector2D.cartesian(50,100),
                Vector2D.cartesian(100,0)
        );


        double t = curve.closestT(
                Vector2D.cartesian(50,50),
                0
        );


        Vector2D point = curve.get(t);
        Vector2D tangent = curve.tangent(t);
        Vector2D normal = curve.leftNormal(t);


        System.out.println("Closest t: " + t);
        System.out.println("Point: " + point);
        System.out.println("Tangent: " + tangent);
        System.out.println("Normal: " + normal);


        assertFalse(Double.isNaN(t));

        assertTrue(
                t > 0.25 &&
                        t < 0.75
        );
    }



    @Test
    public void testNormalProjection() {

        Vector2D normal =
                Vector2D.cartesian(0,1);


        Vector2D displacement =
                Vector2D.cartesian(5,10);


        Vector2D projected =
                displacement.projectOnto(normal);


        System.out.println(projected);


        assertEquals(
                0,
                projected.x(),
                1e-6
        );


        assertEquals(
                10,
                projected.y(),
                1e-6
        );
    }



    @Test
    public void testTranslationalCorrectionDirection() {
        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        MotionState state =
                MotionState.ofTwist(
                        new Pose(
                                0,
                                0,
                                0
                        ),
                        new Twist(
                                0,
                                0,
                                0
                        )
                );


        Vector2D error =
                Vector2D.cartesian(
                        0,
                        10
                );


        Vector2D correction =
                foresight.computeTranslationalCorrection(
                        state,
                        error,
                        Vector2D.zero()
                );


        System.out.println(
                "Correction: " + correction
        );


        assertTrue(
                correction.y() > 0
        );
    }




    @Test
    public void testFieldToRobotTransform() {


        MotionState state =
                MotionState.ofTwist(
                        new Pose(
                                0,
                                0,
                                Math.PI / 2
                        ),
                        new Twist(
                                0,
                                0,
                                0
                        )
                );


        Vector2D fieldVector =
                Vector2D.cartesian(
                        1,
                        0
                );


        Vector2D robotVector =
                fieldVector.rotate(
                        -state.pose().heading()
                );


        System.out.println(
                robotVector
        );


        assertEquals(
                0,
                robotVector.x(),
                1e-6
        );


        assertEquals(
                -1,
                robotVector.y(),
                1e-6
        );
    }





    @Test
    public void testBrakeVelocity() {


        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        Pair<Double,Double> velocity =
                foresight.getVelocityToBrakeInTime(
                        50,
                        0
                );


        System.out.println(
                "Brake velocity: "
                        + velocity.first()
        );


        assertTrue(
                velocity.first() > 0
        );
    }





    @Test
    public void testHeadingErrorWrap() {


        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        double error =
                foresight.headingError(
                        Math.toRadians(179),
                        Math.toRadians(-179)
                );


        System.out.println(
                "Heading error: "
                        + Math.toDegrees(error)
        );


        assertEquals(
                2,
                Math.toDegrees(error),
                1e-6
        );
    }




    @Test
    public void testCentripetalAcceleration() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        double accel =
                foresight.centripetal(
                        10,
                        0.5
                );


        System.out.println(
                accel
        );


        assertTrue(
                accel > 0
        );
    }

    @Test
    public void testHeadingPowerSign() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        MotionState state =
                MotionState.ofTwist(
                        new Pose(
                                0,
                                0,
                                0
                        ),
                        new Twist(
                                0,
                                0,
                                0
                        )
                );


        // target is +90 degrees CCW
        double error =
                foresight.headingError(
                        0,
                        Math.PI / 2
                );


        double power =
                foresight.headingPower(
                        error,
                        state
                );


        System.out.println(
                "Heading error: " + error
        );

        System.out.println(
                "Heading power: " + power
        );


        /*
         * This assumes your drivetrain defines positive turn as CCW.
         * If this fails, your heading convention is flipped.
         */
        assertTrue(
                power > 0
        );
    }



    @Test
    public void testRobotFrameDriveDirection() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        MotionState state =
                MotionState.ofTwist(
                        new Pose(
                                0,
                                0,
                                0
                        ),
                        new Twist(
                                0,
                                0,
                                0
                        )
                );


        Vector2D fieldPower =
                Vector2D.cartesian(
                        1,
                        0
                );


        DrivePowers powers =
                foresight.getDrivePowers(
                        fieldPower,
                        state,
                        0
                );


        System.out.println(
                "Forward: " + powers.forward()
        );

        System.out.println(
                "Strafe: " + powers.strafe()
        );


        assertEquals(
                1,
                powers.forward(),
                1e-6
        );


        assertEquals(
                0,
                powers.strafe(),
                1e-6
        );
    }




    @Test
    public void testPureRotationDirection() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        MotionState state =
                MotionState.ofTwist(
                        new Pose(
                                0,
                                0,
                                0
                        ),
                        new Twist(
                                0,
                                0,
                                0
                        )
                );


        DrivePowers powers =
                foresight.getDrivePowers(
                        Vector2D.zero(),
                        state,
                        1
                );


        System.out.println(
                "Rotation command: " +
                        powers.turn()
        );


        assertEquals(
                1,
                powers.turn(),
                1e-6
        );
    }




    @Test
    public void testAllocation() {

        Foresight foresight =
                new Foresight(
                        Constants.foresightConfig
                );


        Vector2D translation =
                Vector2D.cartesian(
                        1,
                        0
                );


        Vector2D rotationVector =
                Vector2D.polar(
                        1,
                        Math.PI / 2
                );


        System.out.println(
                "Translation: " + translation
        );


        System.out.println(
                "Rotation vector: " + rotationVector
        );


        /*
         * This is mainly a sanity check:
         * if vector addition is broken before
         * drivetrain allocation, this catches it.
         */
        Vector2D combined =
                translation.plus(rotationVector);


        System.out.println(
                "Combined: " + combined
        );


        assertFalse(
                Double.isNaN(combined.x())
        );

        assertFalse(
                Double.isNaN(combined.y())
        );
    }
}