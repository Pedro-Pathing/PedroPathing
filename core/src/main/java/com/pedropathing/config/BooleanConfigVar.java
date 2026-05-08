package com.pedropathing.config;

public class BooleanConfigVar {
    private boolean value;
    private boolean hasValue;

    private BooleanConfigVar(boolean value) {
        this.value = value;
        this.hasValue = true;
    }

    private BooleanConfigVar() {
        this.hasValue = false;
    }

    public static BooleanConfigVar of(boolean value) {
        return new BooleanConfigVar(value);
    }

    public static BooleanConfigVar empty() {
        return new BooleanConfigVar();
    }

    public boolean get() {
        require();
        return value;
    }

    public void set(boolean value) {
        this.value = value;
        this.hasValue = true;
    }

    public void require() {
        if (!hasValue) throw new IllegalStateException("Config variable has not been set");
    }

    public Modifier modify(boolean tempValue) {
        return new Modifier() {
            private boolean originalValue;

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
