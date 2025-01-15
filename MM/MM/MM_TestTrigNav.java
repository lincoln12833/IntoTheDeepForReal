package org.firstinspires.ftc.teamcode.MM.MM;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestTrigNav", group="mm")
@Config
public class MM_TestTrigNav extends MM_OpMode {
    public static double TARGET_X = 0;
    public static double TARGET_Y = -48;
    public static double TARGET_HEADING = 90;
    public static double ROTATE_FACTOR = 0.036;

    @Override
    public void runProcedures(){
        robot.navigation.setPosition(-48, -48, 90);
        while(opModeIsActive()) {
            if(gamepad1.a) {
                robot.drivetrain.driveToPosition(TARGET_X, TARGET_Y, TARGET_HEADING, ROTATE_FACTOR, -25, 0, false, false);//rotateFactor speeds up the rotate and is based on the distance we are moving
            }
        }
    }
}