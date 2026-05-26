package com.pedropathing.paths;

import lombok.Getter;
import lombok.Setter;

import java.util.Deque;
import java.util.ArrayDeque;

public final class PathTracker {
    private final Deque<AtomicPath> atomicPaths;
    
    @Getter @Setter
    private boolean isBusy;

    public PathTracker(Path path) {
        atomicPaths = new ArrayDeque<>(path.getPaths());
        if (!atomicPaths.isEmpty()) isBusy = true;
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