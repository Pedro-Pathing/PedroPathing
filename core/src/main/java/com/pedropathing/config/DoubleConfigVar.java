package com.pedropathing.config;

public class DoubleConfigVar {
    private double value;
    private boolean hasValue;

    private DoubleConfigVar(double value) {
        this.value = value;
        this.hasValue = true;
    }

    private DoubleConfigVar() {
        this.hasValue = false;
    }

    public static DoubleConfigVar of(double value) {
        return new DoubleConfigVar(value);
    }

    public static DoubleConfigVar empty() {
        return new DoubleConfigVar();
    }

    public double get() {
        require();
        return value;
    }

    public void set(double value) {
        this.value = value;
        this.hasValue = true;
    }

    public void require() {
        if (!hasValue) throw new IllegalStateException("Config variable has not been set");
    }

    public Modifier temp(double tempValue) {
        return new Modifier() {
            private double originalValue;

            @Override
            public void apply() {
                originalValue = value;
                value = tempValue;
            }

            @Override
            public void revert() {
                value = originalValue;
            }
        };
    }
}