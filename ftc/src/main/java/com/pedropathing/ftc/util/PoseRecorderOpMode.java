package com.pedropathing.ftc.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A turnkey base OpMode for recording field coordinates by dragging or driving the robot around.
 *
 * <p>Extend this in your TeamCode, add the {@code @TeleOp} annotation, and implement
 * {@link #createFollower(HardwareMap)} to build your configured {@link Follower}. Optionally
 * override {@link #startingPose()} to declare, <em>in code</em>, where you physically place the
 * robot before pressing start. Everything else — teleop driving, gamepad capture, the numbered
 * Driver Station list, and streaming captures to your computer via {@code adb logcat} — is
 * handled by {@link PoseRecorder}.</p>
 *
 * <pre>{@code
 * @TeleOp(name = "Pose Recorder")
 * public class MyPoseRecorder extends PoseRecorderOpMode {
 *     @Override protected Follower createFollower(HardwareMap h) { return Constants.createFollower(h); }
 *     @Override protected Pose startingPose() { return new Pose(9, 111, Math.toRadians(90)); }
 * }
 * }</pre>
 *
 * <p>During the run, drive with the left stick (translate) and right stick (turn), or just push
 * the robot by hand — the localizer tracks its pose either way. Press <b>A</b> to capture the
 * current pose, <b>B</b> to undo, and <b>Back + Y</b> to clear.</p>
 *
 * @author Pedro Pathing
 */
public abstract class PoseRecorderOpMode extends LinearOpMode {

    /**
     * Builds the configured follower for this robot, typically via {@code FollowerBuilder}.
     *
     * @param hardwareMap the OpMode hardware map
     * @return the follower whose pose will be recorded
     */
    protected abstract Follower createFollower(HardwareMap hardwareMap);

    /**
     * The pose the robot is physically placed at before the OpMode starts. Override to declare
     * your starting position in code. Defaults to the origin (0, 0, heading 0).
     *
     * @return the starting pose
     */
    protected Pose startingPose() {
        return new Pose();
    }

    /**
     * Whether to enable gamepad teleop driving. Override to return {@code false} for a purely
     * hand-dragged workflow with the motors left off. Defaults to {@code true}.
     *
     * @return true to enable teleop driving
     */
    protected boolean useTeleOpDrive() {
        return true;
    }

    @Override
    public void runOpMode() {
        Follower follower = createFollower(hardwareMap);
        // setStartingPose must run before any movement, so it happens here in init.
        follower.setStartingPose(startingPose());

        PoseRecorder recorder = new PoseRecorder(follower, gamepad1, telemetry);

        telemetry.addLine("Pose Recorder ready. Press start, then drag/drive the robot.");
        telemetry.addData("Starting pose", startingPose());
        telemetry.update();

        waitForStart();

        if (useTeleOpDrive()) {
            follower.startTeleopDrive();
        }

        while (opModeIsActive()) {
            if (useTeleOpDrive()) {
                // Library convention: forward = -left_stick_y, strafe = -left_stick_x,
                // turn = -right_stick_x. Robot-centric so pushing the robot by hand is unaffected.
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true);
            }

            follower.update();
            recorder.update();
            telemetry.update();
        }
    }
}
