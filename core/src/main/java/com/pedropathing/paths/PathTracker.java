/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.math.Pose;
import java.util.ArrayDeque;
import java.util.Deque;

public final class PathTracker {
    private final Deque<AtomicPath> atomicPaths;

    public PathTracker(Path path) {
        atomicPaths = new ArrayDeque<>(path.getPaths());

        Path lastPath = atomicPaths.getLast();
        endPose = lastPath.curve.get(1).toPose(lastPath.heading(1));
    }

    private final Pose endPose;

    public Pose endPose() {
        return endPose;
    }

    public void advance() {
        if (atomicPaths.isEmpty()) throw new IllegalStateException("Cannot advance past last path");
        atomicPaths.remove();
    }

    public Path current() {
        return atomicPaths.peek();
    }

    public boolean done() {
        return atomicPaths.isEmpty();
    }

    public int remainingPaths() {
        return atomicPaths.size();
    }
}
