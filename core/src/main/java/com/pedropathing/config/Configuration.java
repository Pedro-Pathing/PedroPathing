package com.pedropathing.config;

@FunctionalInterface
public interface Configuration<T> {
    void configure(T config);
}
