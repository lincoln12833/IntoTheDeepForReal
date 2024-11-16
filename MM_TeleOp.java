package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="mm")
public class MM_TeleOp extends MM_OpMode {
    @Override
    public void runProcedures(){
        while(opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            robot.drivetrain.driveWithSticks();
            robot.transport.runTransport();
            robot.collector.handleCollect(true);

            telemetry.update();
        }

    }

}
