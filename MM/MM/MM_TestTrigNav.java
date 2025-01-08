package org.firstinspires.ftc.teamcode.MM.MM;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="TestTrigNav", group="mm")
@Config
public class MM_TestTrigNav extends MM_OpMode {
    @Override
    public void runProcedures(){
        robot.navigation.setPosition(-52, -51, 55);
        robot.drivetrain.driveToPosition(-65, -48, 45, .016, 90, 52, false, false); //rotateFactor speeds up the rotate and is based on the distance we are moving
    }
}