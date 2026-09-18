package com.pedropathing.revhub.localizers;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

final class CountingIntSupplier implements IntSupplier {
    int value;
    int calls;

    CountingIntSupplier() {
        this(0);
    }

    CountingIntSupplier(int value) {
        this.value = value;
    }

    @Override
    public int getAsInt() {
        calls++;
        return value;
    }
}

final class CountingDoubleSupplier implements DoubleSupplier {
    double value;
    int calls;

    CountingDoubleSupplier() {
        this(0);
    }

    CountingDoubleSupplier(double value) {
        this.value = value;
    }

    @Override
    public double getAsDouble() {
        calls++;
        return value;
    }
}
