/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import java.util.ArrayDeque;
import java.util.Deque;

import com.pedropathing.math.Pose;
import lombok.Getter;
import lombok.Setter;

public final class PathTracker {
    private final Deque<AtomicPath> atomicPaths;

    @Getter
    @Setter
    private boolean isFollowing;

    @Getter
    private Pose end;

    public PathTracker(Path path) {
        atomicPaths = new ArrayDeque<>(path.getPaths());
        if (!atomicPaths.isEmpty()) {
            Path lastPath = atomicPaths.peekLast();
            end = lastPath.endPoint().toPose(lastPath.heading(1));
            isFollowing = true;
        }
    }

    public void advance() {
        atomicPaths.remove();
    }

    public Path current() {
        return atomicPaths.peek();
    }

    public boolean empty() {
        return atomicPaths.isEmpty();
    }

    public int size() {
        return atomicPaths.size();
    }
}
