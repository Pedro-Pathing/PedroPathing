/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.tvalue;

import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.annotation.Before;

@Aspect
public class TValueCheckAspect {
    private static void checkT(double t) {
        if (t < 0 || t > 1) throw new IllegalArgumentException("t must be between 0 and 1 but was " + t + ".");
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(@com.pedropathing.paths.tvalue.TValue (double), ..)) && args(t, ..)",
            argNames = "t")
    public void checkTValueAt1(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, t, ..)",
            argNames = "t")
    public void checkTValueAt2(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, t, ..)",
            argNames = "t")
    public void checkTValueAt3(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, *, t, ..)",
            argNames = "t")
    public void checkTValueAt4(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, *, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, *, *, t, ..)",
            argNames = "t")
    public void checkTValueAt5(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, *, *, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, *, *, *, t, ..)",
            argNames = "t")
    public void checkTValueAt6(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, *, *, *, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, *, *, *, *, t, ..)",
            argNames = "t")
    public void checkTValueAt7(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, *, *, *, *, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, *, *, *, *, *, t, ..)",
            argNames = "t")
    public void checkTValueAt8(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, *, *, *, *, *, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, *, *, *, *, *, *, t, ..)",
            argNames = "t")
    public void checkTValueAt9(double t) {
        checkT(t);
    }

    @Before(
            value =
                    "execution(* com.pedropathing.paths..*(*, *, *, *, *, *, *, *, *, @com.pedropathing.paths.tvalue.TValue (double), ..)) && args(*, *, *, *, *, *, *, *, *, t, ..)",
            argNames = "t")
    public void checkTValueAt10(double t) {
        checkT(t);
    }
}
