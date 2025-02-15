package com.pedropathing.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class PedroOpMode extends LinearOpMode {
    Follower follower;
    public boolean holdEnd = true;
    /// List of linear auto actions before their path chain
    public List<AutoActions> beforePTLinear = new ArrayList();
    /// List of linear auto actions after their path chain
    public List<AutoActions> afterPTLinear = new ArrayList();
    /// List of parallel auto actions after their path chain
    public List<AutoActions> beforePTParallel = new ArrayList();
    /// List of parallel auto actions after their path chain
    public List<AutoActions> afterPTParallel = new ArrayList();
    /// Thread for telemetry, won't disturb main thread
    Thread telemetryThread = new Thread(() -> {
        while (opModeIsActive()) {
            telemetryDebug();
            telemetry.update();
        }
    });
    // shouldn't be used
    @Override
    public void runOpMode() throws InterruptedException {
        initMechanism();
        waitForStart();
        afterWaitForStart();
        telemetryThread.start();
        while (opModeIsActive()) {
            play();
        }
    }
    /// runs after waitForStart()
    public abstract void afterWaitForStart();
    /// runs in init
    public abstract void initMechanism();
    /// runs while opModeIsActive
    public abstract void play();
    /// runs action while opModeIsActive in different threads
    public void playActionOpMode(TeleOpActions... teleOpActions){
        for(int i = 0; i < teleOpActions.length;i++){
            teleOpActions[i].setOpMode(this);
            teleOpActions[i].start();
        }
    }
    /// updates follower
    Thread fUpdate = new Thread(() -> {
        while (opModeIsActive()) {
            follower.update();
        }
    });
    /// adds linear actions to list
    public void beforePTLinear(AutoActions... autoActions){
        for (AutoActions autoAction : autoActions) {
            beforePTLinear.add(autoAction.pathNumber, autoAction);
        }
    }
    /// adds parallel actions to list
    public void beforePTParallel(AutoActions... autoActions){
        for (AutoActions autoAction : autoActions) {
            beforePTParallel.add(autoAction.pathNumber, autoAction);
        }
    }
    /// adds linear actions to list
    public void afterPTLinear(AutoActions... autoActions){
        for (AutoActions autoAction : autoActions) {
            afterPTLinear.add(autoAction.pathNumber, autoAction);
        }
    }
    /// adds parallel actions to list
    public void afterPTParallel(AutoActions... autoActions){
        for (AutoActions autoAction : autoActions) {
            afterPTParallel.add(autoAction.pathNumber, autoAction);
        }
    }
    ///  Follows pathChains
    public void pedroFollowOpMode(Follower follower, PathChain... pathChains){
        // adds follower
        this.follower = follower;
        fUpdate.start();
        for (int i = 0; i < pathChains.length; i++) {
            if (pathChains[i] != null) {
                if(i == 0){
                    if (beforePTLinear.get(i) != null) {
                        beforePTLinear.get(i).linearAction();
                    }
                    if (beforePTParallel.get(i) != null) {
                        beforePTLinear.get(i).start();
                    }
                    follower.followPath(pathChains[i], holdEnd);
                    if (afterPTLinear.get(i) != null) {
                        afterPTLinear.get(i).linearAction();
                    }
                    if (afterPTParallel.get(i) != null){
                        afterPTParallel.get(i).start();
                    }

                }
                else {
                    if(!follower.isBusy()&& (follower.getPose().getX() > (follower.getCurrentPath().getLastControlPoint().getX() - 1) && follower.getPose().getY() > (follower.getCurrentPath().getLastControlPoint().getY() - 1) )|| follower.isRobotStuck()){
                        if(holdEnd){
                            follower.holdPoint(follower.getCurrentPath().getLastControlPoint(), follower.getCurrentPath().getClosestPointHeadingGoal());
                        }
                        if (beforePTLinear.get(i) != null) {
                            beforePTLinear.get(i).linearAction();
                        }
                        if (beforePTParallel.get(i) != null) {
                            beforePTLinear.get(i).start();
                        }
                        follower.followPath(pathChains[i], holdEnd);
                        if (afterPTLinear.get(i) != null) {
                            afterPTLinear.get(i).linearAction();
                        }
                        if (afterPTParallel.get(i) != null){
                            afterPTParallel.get(i).start();
                        }
                    }
                }
            }
            else{
                break;
            }
        }
    }
    ///adds telemetry
    public abstract void telemetryDebug();
    ///building paths
    public abstract void pathBuilding();
}
