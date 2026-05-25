package com.pedropathing.revhub.drivetrains;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Test {
    public static void main(String[] args) {
        MecanumConfig config = new MecanumConfig(
                c -> {
                    c.leftFrontName.set("leftFront");
                    c.leftRearName.set("leftRear");
                    c.rightFrontName.set("rightFront");
                    c.rightRearName.set("rightRear");

                    c.leftFrontDirection.set(DcMotorSimple.Direction.FORWARD);
                    c.leftRearDirection.set(DcMotorSimple.Direction.FORWARD);
                    c.rightFrontDirection.set(DcMotorSimple.Direction.REVERSE);
                    c.rightRearDirection.set(DcMotorSimple.Direction.REVERSE);

                    c.manualBrakeMode.set(true);
                }
        );

        PinpointConfig pinpointConfig = new PinpointConfig(
                c -> {
                    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

                    c.xPodOffset.set(0.0);
                    c.yPodOffset.set(0.0);

                    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                }
        );
    }
}
