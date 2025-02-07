package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="fixString", group="test")
public class fixString extends MM_OpMode{
    @Override
    public void runProcedures(){
        while(opModeIsActive()) {
            robot.ascent.fixString();
        }
    }
}
