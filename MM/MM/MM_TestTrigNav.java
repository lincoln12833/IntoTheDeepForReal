package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.DRIVE_CONSTANTS.BASE_ROTATE_FACTOR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestTrigNav", group="mm")
public class MM_TestTrigNav extends MM_OpMode {
    public static double TARGET_X = 0;
    public static double TARGET_Y = -48;
    public static double TARGET_HEADING = 90;


    @Override
    public void runProcedures(){
        robot.navigation.setPosition(-48, -48, 90);
        while(opModeIsActive()) {
            if(gamepad1.a) {
                //robot.drivetrain.driveToPosition(TARGET_X, TARGET_Y, .6, TARGET_HEADING, BASE_ROTATE_FACTOR, -25, 0, false, false);//rotateFactor speeds up the rotate and is based on the distance we are moving
            }
        }
    }
}