package com.pedropathing.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class PedroOpMode extends LinearOpMode {
    Follower follower;
    public double pathState = 0;
    public boolean holdEnd = true;
    private Timer pathTimer, actionTimer;

    public List<AutoActions> beforePTLinear = new ArrayList();

    public List<AutoActions> afterPTLinear = new ArrayList();
    public List<AutoActions> beforePTParallel = new ArrayList();
    public List<AutoActions> afterPTParallel = new ArrayList();
    Thread telemetryThread = new Thread(() -> {
        while (opModeIsActive()) {
            telemetryDebug();
            telemetry.update();
        }
    });
    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        actionTimer = new Timer();
        initMechanism();
        waitForStart();
        afterWaitForStart();
        telemetryThread.start();
        while (opModeIsActive()) {
            play();
        }
    }
    public abstract void afterWaitForStart();
    public abstract void initMechanism();
    public abstract void play();
    public void playActionOpMode(TeleOpActions... teleOpActions){
        for(int i = 0; i < teleOpActions.length;i++){
            teleOpActions[i].setOpMode(this);
            teleOpActions[i].start();
        }
    }

    Thread fUpdate = new Thread(() -> {
        while (opModeIsActive()) {
            follower.update();
        }
    });
    public void beforePTLinear(AutoActions... autoActions){
        beforePTLinear.addAll(Arrays.asList(autoActions));
    }
    public void beforePTParallel(AutoActions... autoActions){
        beforePTParallel.addAll(Arrays.asList(autoActions));
    }
    public void afterPTLinear(AutoActions... autoActions){
        afterPTLinear.addAll(Arrays.asList(autoActions));
    }
    public void afterPTParallel(AutoActions... autoActions){
        afterPTParallel.addAll(Arrays.asList(autoActions));
    }
    public void pedroFollowOpMode(Follower follower, PathChain... pathChains){
        this.follower = follower;
        fUpdate.start();
        for (int i = 0; i < pathChains.length; i++) {
            if (pathChains[i] != null) {
                if(i == 0){
                    beforePTLinear.get(i).linearAction();
                    beforePTParallel.get(i).start();
                    follower.followPath(pathChains[i], holdEnd);
                    afterPTLinear.get(i).linearAction();
                    afterPTParallel.get(i).start();
                }
                else {
                    if(!follower.isBusy()&& (follower.getPose().getX() > (follower.getCurrentPath().getLastControlPoint().getX() - 1) && follower.getPose().getY() > (follower.getCurrentPath().getLastControlPoint().getY() - 1) )|| follower.isRobotStuck()){
                        beforePTLinear.get(i).linearAction();
                        beforePTParallel.get(i).start();
                        follower.followPath(pathChains[i], holdEnd);
                        afterPTLinear.get(i).linearAction();
                        afterPTParallel.get(i).start();
                    }
                }
            }
            else{
                break;
            }
        }
    }
    public abstract void telemetryDebug();
    public abstract void pathBuilding();
}
