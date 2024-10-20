package org.firstinspires.ftc.teamcode.MM;

public class MM_Robot{
    private final MM_OpMode opMode;
    
    public MM_Drivetrain drivetrain;

    MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
    }
}
