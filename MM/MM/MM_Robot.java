package org.firstinspires.ftc.teamcode.MM.MM;

public class MM_Robot{
    private final MM_OpMode opMode;
    
    public MM_Drivetrain drivetrain;
    public MM_Transport transport;
    public MM_Collector collector;
    public MM_VisionPortal visionPortal;

    public static double robotX;
    public static double robotY;



    MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }



    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        transport = new MM_Transport(opMode);
        collector = new MM_Collector(opMode);
        visionPortal = new MM_VisionPortal(opMode);
    }


}
