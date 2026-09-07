package com.pedropathing.revhub;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class Loggers {
    public static Consumer<String> telemetryLog(Telemetry telemetry) {
        return (message) -> telemetry.addData("Pedro Pathing Log", message);
    }

    public static Consumer<String> logger(BiConsumer<String, String> logger) {
        return (message) -> logger.accept("Pedro Pathing Log", message);
    }
}
