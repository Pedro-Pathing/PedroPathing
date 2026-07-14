package com.pedropathing.ftc.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * A reusable helper for interactively recording robot poses off a {@link Follower}.
 *
 * <p>Drop this into any TeleOp OpMode: construct it once with your {@code Follower}, a
 * {@code Gamepad}, and the OpMode's {@code Telemetry}, then call {@link #update()} every loop
 * iteration (after {@code follower.update()}). While the OpMode runs you physically drag or
 * drive the robot around the field and press a button to capture wherever it currently is. Each
 * captured {@link Pose} is:</p>
 *
 * <ul>
 *     <li>appended to an in-memory list, shown on the Driver Station as a numbered,
 *     copy-pasteable {@code new Pose(...)} snippet, and</li>
 *     <li>written to the Robot Controller log via {@link RobotLog}, so it streams to your coding
 *     computer through {@code adb logcat} (filter tag {@code PoseRecorder}) and is saved to the
 *     RC log file.</li>
 * </ul>
 *
 * <p>Poses are captured in the follower's native (Pedro) coordinate system, so the emitted
 * snippets drop straight into path-building code.</p>
 *
 * <p>Default button map (all reconfigurable via the setters): <b>A</b> captures the current
 * pose, <b>B</b> undoes the last capture, and <b>Back + Y</b> clears the whole list (the guard
 * button avoids wiping your work by accident).</p>
 *
 * @author Pedro Pathing
 */
public class PoseRecorder {
    /** A gamepad button, resolved against a {@link Gamepad} each loop for rising-edge detection. */
    public interface Button {
        boolean isPressed(Gamepad gamepad);
    }

    private final Follower follower;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    private final List<Pose> recorded = new ArrayList<>();

    private Button captureButton = g -> g.a;
    private Button undoButton = g -> g.b;
    private Button clearButton = g -> g.y;
    private Button clearGuard = g -> g.back;

    private boolean previousCapture = false;
    private boolean previousUndo = false;
    private boolean previousClear = false;

    /**
     * @param follower  the follower whose current pose will be captured
     * @param gamepad   the gamepad polled for capture/undo/clear buttons
     * @param telemetry the OpMode telemetry used to render the live pose and recorded list
     */
    public PoseRecorder(Follower follower, Gamepad gamepad, Telemetry telemetry) {
        this.follower = follower;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    /** Sets the button that captures the current pose (default {@code A}). */
    public PoseRecorder setCaptureButton(Button captureButton) {
        this.captureButton = captureButton;
        return this;
    }

    /** Sets the button that removes the most recent capture (default {@code B}). */
    public PoseRecorder setUndoButton(Button undoButton) {
        this.undoButton = undoButton;
        return this;
    }

    /** Sets the button that clears every capture (default {@code Y}, guarded). */
    public PoseRecorder setClearButton(Button clearButton) {
        this.clearButton = clearButton;
        return this;
    }

    /**
     * Sets the guard button that must be held together with the clear button (default
     * {@code Back}). Pass {@code g -> true} to disable the guard.
     */
    public PoseRecorder setClearGuard(Button clearGuard) {
        this.clearGuard = clearGuard;
        return this;
    }

    /**
     * Polls the gamepad, applies any capture/undo/clear action on the button's rising edge, and
     * renders telemetry. Call once per loop iteration, after {@code follower.update()}.
     * Does not call {@code telemetry.update()} — the OpMode owns that.
     */
    public void update() {
        boolean capture = captureButton.isPressed(gamepad);
        boolean undo = undoButton.isPressed(gamepad);
        boolean clear = clearButton.isPressed(gamepad) && clearGuard.isPressed(gamepad);

        if (capture && !previousCapture) {
            capture();
        }
        if (undo && !previousUndo) {
            undo();
        }
        if (clear && !previousClear) {
            clear();
        }

        previousCapture = capture;
        previousUndo = undo;
        previousClear = clear;

        renderTelemetry();
    }

    /** Captures the follower's current pose and logs it. */
    public void capture() {
        Pose pose = follower.getPose().copy();
        recorded.add(pose);
        RobotLog.ii("PoseRecorder", "captured #%d: %s", recorded.size(), snippet(pose));
    }

    /** Removes the most recent capture, if any. */
    public void undo() {
        if (!recorded.isEmpty()) {
            Pose removed = recorded.remove(recorded.size() - 1);
            RobotLog.ii("PoseRecorder", "undo: removed %s (%d left)", snippet(removed), recorded.size());
        }
    }

    /** Removes every capture. */
    public void clear() {
        int count = recorded.size();
        recorded.clear();
        RobotLog.ii("PoseRecorder", "cleared %d pose(s)", count);
    }

    /**
     * Returns a copy of the recorded poses in capture order. Mutating the returned list does not
     * affect the recorder.
     */
    public List<Pose> getRecordedPoses() {
        return new ArrayList<>(recorded);
    }

    private void renderTelemetry() {
        Pose current = follower.getPose();
        telemetry.addLine("== Pose Recorder ==");
        telemetry.addData("Current", "x %.2f  y %.2f  heading %.2f°",
                current.getX(), current.getY(), Math.toDegrees(current.getHeading()));
        telemetry.addLine("A: capture   B: undo   Back+Y: clear");
        telemetry.addData("Recorded", recorded.size());
        for (int i = 0; i < recorded.size(); i++) {
            telemetry.addLine((i + 1) + ": " + snippet(recorded.get(i)));
        }
    }

    /** Formats a pose as a copy-pasteable {@code new Pose(x, y, Math.toRadians(deg))} snippet. */
    private static String snippet(Pose pose) {
        return String.format("new Pose(%.3f, %.3f, Math.toRadians(%.2f))",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
