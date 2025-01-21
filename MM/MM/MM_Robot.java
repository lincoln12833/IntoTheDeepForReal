package org.firstinspires.ftc.teamcode.MM.MM;

public class MM_Robot{
    private final MM_OpMode opMode;
    
    public MM_Drivetrain drivetrain;
    public MM_Transport transport;
    public MM_Collector collector;
    public MM_Navigation navigation;
    public MM_Ascent ascent;

    MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        transport = new MM_Transport(opMode);
        collector = new MM_Collector(opMode);
        navigation = new MM_Navigation(opMode);
        ascent = new MM_Ascent(opMode);
    }
}