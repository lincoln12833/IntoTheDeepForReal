package org.firstinspires.ftc.teamcode.IntoTheDeepForReal;

public class MM_Robot{
    private final MM_OpMode opMode;
    
    //public MM_Drivetrain drivetrain;
    public MM_Transport.MM_Slide slide;
    public MM_Transport.MM_Pivot pivot;

    MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        //drivetrain = new MM_Drivetrain(opMode);
        slide = new MM_Transport.MM_Slide(opMode);
        pivot = new MM_Transport.MM_Pivot(opMode);
    }
}
