/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.paths.tvalue.TValue;
import java.util.*;
import java.util.function.ToDoubleFunction;
import lombok.Value;

public final class Piecewise<T> {
    private final ToDoubleFunction<T> getLength;
    private final double totalLength;
    private final List<Segment<T>> segments = new ArrayList<>();
    private final NavigableMap<Double, Segment<T>> segmentMap = new TreeMap<>();

    public Piecewise(ToDoubleFunction<T> getLength, T[] items) {
        if (items.length == 0) throw new IllegalArgumentException("Piecewise must have at least one item.");

        this.getLength = getLength;
        totalLength = Arrays.stream(items).mapToDouble(getLength).sum();

        double currentT = 0.0;
        for (T item : items) {
            Segment<T> segment = new Segment<>(currentT, item);
            segments.add(segment);
            segmentMap.put(currentT, segment);
            currentT += getLength.applyAsDouble(item) / totalLength;
        }
    }

    public T get(@TValue double t) {
        return getSegment(t).value();
    }

    public Segment<T> getSegment(@TValue double t) {
        return segmentMap.floorEntry(t).getValue();
    }

    public double localT(@TValue double t) {
        Segment<T> segment = getSegment(t);
        return (t - segment.startT()) / getLength.applyAsDouble(segment.value()) * totalLength;
    }

    public double globalT(Segment<T> segment, @TValue double localT) {
        return segment.startT() + localT * getLength.applyAsDouble(segment.value()) / totalLength;
    }

    public double length() {
        return totalLength;
    }

    public List<Segment<T>> segments() {
        return Collections.unmodifiableList(segments);
    }

    @Value
    public static class Segment<T> {
        double startT;
        T value;
    }
}
