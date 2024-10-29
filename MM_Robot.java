package org.firstinspires.ftc.teamcode.MM;

public class MM_Robot{
    private final MM_OpMode opMode;
    
    public MM_Drivetrain drivetrain;

    public MM_Pivot pivot;

    MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        pivot = new MM_Pivot(opMode);
    }
}
