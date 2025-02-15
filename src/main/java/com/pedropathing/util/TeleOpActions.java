package com.pedropathing.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class TeleOpActions extends Thread{
    LinearOpMode opMode;

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void run() {
        super.run();
        while(!(opMode.isStopRequested())){
            play();
        }
    }

    public abstract void play();
}
