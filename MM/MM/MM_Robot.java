package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot{
    private final MM_OpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Transport transport;
    public MM_Collector collector;
    public MM_Navigation navigation;
    public MM_Ascent ascent;

    public boolean scoring;
    public boolean travelling;
    public boolean collecting;
    public boolean slideTargetSet;
    public boolean collectTimeisStarted;
    public ElapsedTime scoreTimer = new ElapsedTime();
    public ElapsedTime collectTimer = new ElapsedTime();

    MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void scoreSpecimen(){
        transport.slide.setTargetTicks(1350);
        while(!transport.slide.slideMovementDone()){}
        collector.scoreSpec();
        opMode.sleep(500);
    }

    public void doSpec(){
       if (opMode.gamepad1.y || collecting){
           if(!collecting) {
               drivetrain.driveToDistance(4.7);
               collecting = true;
           } else {
               teleCollectSpec();
           }
//            drivetrain.driveToPosition(-4, -45, .4, 0, alliance == 1? -90: -90 + 180, BASE_ROTATE_FACTOR, -1, 92, 16.1, false, false);
        }
            if (opMode.gamepad1.x || scoring){
            if(!scoring) {
                drivetrain.driveToDistance(5.9);
            } else {
                teleScoreSpec();
            }

        }
    }

    public void teleScoreSpec(){
        if(!slideTargetSet) {
            transport.slide.setTargetTicks(1350);
            transport.slide.slideTargetTicks = 1350;
            slideTargetSet = true;
        }
        if(transport.slide.slideMovementDone()){
            collector.openSpecClaw();
            scoring = false;
            slideTargetSet = false;
        }
    }
    public void collectSpec(){
        collector.collectSpec();
        opMode.sleep(1000);
        transport.slide.setTargetTicks(1828);
    }

    public void clearCollector() {
        if(!collector.haveSample()){
            collector.discardSample();
        }
    }

    public void teleCollectSpec(){
        collector.collectSpec();
        if (!collectTimeisStarted){
            collectTimer.reset();
            collectTimeisStarted = true;
        }
        if(collectTimer.milliseconds() > 1000 && collectTimeisStarted){
            transport.slide.setTargetTicks(1828);
            transport.slide.slideTargetTicks = 1828;
            collecting = false;
        }
    }


    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        transport = new MM_Transport(opMode);
        collector = new MM_Collector(opMode);
        navigation = new MM_Navigation(opMode);
        ascent = new MM_Ascent(opMode);
    }
}