package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="testSensor", group="test")
public class MM_TestSensor extends MM_OpMode {
    @Override
    public void runProcedures(){
        while(opModeIsActive()){
            robot.collector.getSensorStuff();
        }
    }
}