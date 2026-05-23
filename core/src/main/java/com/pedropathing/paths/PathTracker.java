package com.pedropathing.paths;

import java.util.Deque;
import java.util.ArrayDeque;

public final class PathTracker {
    private final Path path;
    private final Deque<AtomicPath> atomicPaths;

    public PathTracker(Path path) {
        this.path = path;
        atomicPaths = new ArrayDeque<>(path.getPaths());
    }

    public void advancePath() {
        atomicPaths.remove();
    }
}