package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TunePID", group="test")
public class TunePID extends MM_OpMode{

    @Override
    public void runOpMode(){
        while(opModeIsActive()){
            robot.drivetrain.driveToPosition(48, -48, 1,90, 0.037, -24, .5, false, false);
            robot.drivetrain.driveToPosition(48, 48, 1,90, 0.037, -24, .5, false, false);

        }
    }
}
