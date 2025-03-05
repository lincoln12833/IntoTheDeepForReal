package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.DRIVE_CONSTANTS.BASE_ROTATE_FACTOR;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.alliance;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot{
    private final MM_OpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Transport transport;
    public MM_Collector collector;
    public MM_Navigation navigation;
    public MM_Ascent ascent;

    public boolean scoring;
    public boolean collecting;
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
        if(opMode.gamepad1.a || collecting){
            if(!collecting) {
                drivetrain.driveToPosition(55, -56, .8, 0, alliance == 1 ? 180 : 0, BASE_ROTATE_FACTOR, -1, 92, 0, false, false);
                //robot.drivetrain.driveToPosition(46.76, -57, .8, 0, alliance == 1? 90: 90 + 180, BASE_ROTATE_FACTOR, -1, 92,0, false, false);
                drivetrain.driveToDistance(4.7, .8);
                collecting = true;
            } else {
                teleCollectSpec();
            }

        } else if (opMode.gamepad1.y){
            drivetrain.driveToPosition(-4, -45, .4, 0, alliance == 1? -90: -90 + 180, BASE_ROTATE_FACTOR, -1, 92, 16.1, false, false);
        } else if (opMode.gamepad1.x || scoring){
            if(!scoring) {
                drivetrain.driveToDistance(4.7, .6);
                scoring = true;
            } else {
                teleScoreSpec();
            }

        }
    }

    public void teleScoreSpec(){
        transport.slide.setTargetTicks(1350);
        scoreTimer.reset();
        if(scoreTimer.milliseconds() > 1000 && scoreTimer.milliseconds() < 2000){
            collector.teleScoreSpec();
            scoring = false;
        }
    }
    public void collectSpec(){
        collector.collectSpec();
        opMode.sleep(1000);
        transport.slide.setTargetTicks(1828);
    }

    public void teleCollectSpec(){
        collector.collectSpec();
        if(collectTimer.milliseconds() > 1000 && collectTimer.milliseconds() < 2500){
            transport.slide.setTargetTicks(1828);
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