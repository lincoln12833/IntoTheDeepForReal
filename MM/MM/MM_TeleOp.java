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
            robot.collector.sensorTelemetry();
            robot.navigation.updatePosition();
            robot.collector.runSpecClaw();
            robot.ascent.controlAscent();
            robot.doSpec();

            multipleTelemetry.update();


        }
    }
}