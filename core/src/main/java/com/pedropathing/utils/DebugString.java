package com.pedropathing.utils;

import com.pedropathing.follower.Follower;

public class DebugString {
    public String[] debugString;
    private final Follower.Mode mode;

    public DebugString(String localizerInfo, String followInfo, String algorithmInfo, String drivetrainInfo, Follower.Mode mode) {
        debugString = new String[] {localizerInfo, followInfo, algorithmInfo, drivetrainInfo};
        this.mode = mode;
    }

    private static String indent(String string) {
        return "    " + string.replace("\n", "\n    ");
    }

    public String format() {
        StringBuilder output = new StringBuilder();

        output.append("Follower {\n")
                .append(indent(debugString[1]))
                .append("\n},\n");

        output.append("Localizer {\n")
                .append(indent(debugString[0]))
                .append("\n},\n");

        if (!mode.equals(Follower.Mode.MANUAL) && !mode.equals(Follower.Mode.IDLE)) {
            output.append("Algorithm {\n")
                    .append(indent(debugString[2]))
                    .append("\n},\n");
        }

        output.append("Drivetrain {\n")
                .append(indent(debugString[3]))
                .append("\n}");

        return output.toString();
    }
}
