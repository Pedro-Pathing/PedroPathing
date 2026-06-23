package com.pedropathing.utils;

import java.util.concurrent.TimeUnit;

/**
 * This is the Timer class. It is timer with nanosecond precision using System.nanotime()
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/23/26
 */
public class Timer {
    private long startTime;

    /** This creates a new Timer */
    public Timer() {
        reset();
    }

    /** This resets the Timer's start time to the current time */
    public void reset() {
        startTime = System.nanoTime();
    }

    /** This returns the elapsed time in nanoseconds */
    public long nanoseconds() {
        return System.nanoTime() - startTime;
    }

    /**
     * This returns the elapsed time in the given time unit
     * @param unit the time unit to convert to
     */
    public double get(TimeUnit unit) {
        return unit.convert(nanoseconds(), TimeUnit.NANOSECONDS);
    }

    /** This returns the elapsed time in milliseconds */
    public double milliseconds() {
        return get(TimeUnit.SECONDS);
    }

    /** This returns the elapsed time in seconds */
    public double seconds() {
        return get(TimeUnit.SECONDS);
    }

    /** This returns the elapsed time in nanoseconds */
    public double ns() {
        return nanoseconds();
    }

    /** This returns the elapsed time in milliseconds */
    public double ms() {
        return milliseconds();
    }

    /** This returns the elapsed time in seconds */
    public double s() {
        return seconds();
    }
}
