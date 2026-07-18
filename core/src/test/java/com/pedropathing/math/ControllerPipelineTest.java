package com.pedropathing.math;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class ControllerPipelineTest {

    private static void test(Pose robot, Pose target) {

        Vector2D worldError = target.minus(robot).toVector2D();
        Vector2D robotError = worldError.rotate(-robot.heading());

        System.out.println("========================================");
        System.out.println("Robot  : " + robot);
        System.out.println("Target : " + target);

        System.out.printf("World Error : (%.3f, %.3f)%n",
                worldError.x(), worldError.y());

        System.out.printf("Robot Error : (%.3f, %.3f)%n",
                robotError.x(), robotError.y());

        System.out.println();

        if (Math.abs(robotError.x()) > 1e-6) {
            System.out.println(
                    robotError.x() > 0
                            ? "EXPECT: Drive FORWARD"
                            : "EXPECT: Drive BACKWARD");
        }

        if (Math.abs(robotError.y()) > 1e-6) {
            System.out.println(
                    robotError.y() > 0
                            ? "EXPECT: Strafe LEFT"
                            : "EXPECT: Strafe RIGHT");
        }

        double headingError =
                target.heading() - robot.heading();

        System.out.printf("Heading Error : %.1f deg%n",
                Math.toDegrees(headingError));

        if (Math.abs(headingError) > 1e-6) {
            System.out.println(
                    headingError > 0
                            ? "EXPECT: Rotate CCW"
                            : "EXPECT: Rotate CW");
        }

        System.out.println();
    }

    public static void main(String[] args) {

        // Facing east
        test(
                new Pose(0,0,0),
                new Pose(10,0,0));

        test(
                new Pose(0,0,0),
                new Pose(0,10,0));

        test(
                new Pose(0,0,0),
                new Pose(0,-10,0));

        // Facing north
        test(
                new Pose(0,0,Math.toRadians(90)),
                new Pose(10,0,Math.toRadians(90)));

        test(
                new Pose(0,0,Math.toRadians(90)),
                new Pose(0,10,Math.toRadians(90)));

        test(
                new Pose(0,0,Math.toRadians(90)),
                new Pose(-10,0,Math.toRadians(90)));

        // Heading only
        test(
                new Pose(0,0,0),
                new Pose(0,0,Math.toRadians(90)));

        test(
                new Pose(0,0,0),
                new Pose(0,0,Math.toRadians(-90)));

        // Combined
        test(
                new Pose(5,5,Math.toRadians(30)),
                new Pose(10,12,Math.toRadians(90)));
    }
}