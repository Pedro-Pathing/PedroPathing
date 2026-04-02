package com.pedropathing.suppliers.fools;

@FunctionalInterface
public interface PedroSupplier<T> {
    T get();
}

