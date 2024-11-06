package org.firstinspires.ftc.teamcode.IntoTheDeepForReal;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="mm", name="chamber")

public class MM_Autos_Chamber extends MM_OpMode {
    private final ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode(){
        runTime.startTime();

        waitForStart();

        //robot.drivetrain.driveInches(30, 0);
        
    }
    
}
