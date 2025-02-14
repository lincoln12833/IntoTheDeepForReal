package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "fixServos", group = "test")
public class MM_FIX_SERVOS extends MM_OpMode{
    @Override
    public void runProcedures() {
        robot.ascent.liftLeft.setPosition(.5);
        robot.ascent.liftRight.setPosition(.5);

        while(opModeIsActive()){}
    }
}
