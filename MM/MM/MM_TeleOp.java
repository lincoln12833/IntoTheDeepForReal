package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="mm")
public class MM_TeleOp extends MM_OpMode {
    @Override
    public void runProcedures(){
        while(opModeIsActive()){
            previousGamepad1.copy(currentGamepad1); //gamepad management
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            robot.drivetrain.driveWithSticks(); //control mechanisms
            robot.transport.runTransport();
            robot.collector.controlCollector();
            robot.navigation.updatePosition();
            if (currentGamepad1.y && !previousGamepad1.y) {
                robot.drivetrain.driveToPosition(-46.6, -42.6, 91.5, .036, -13.2, 0, false, true);
            }

            multipleTelemetry.update();
        }
    }
}