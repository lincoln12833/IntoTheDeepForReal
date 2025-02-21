package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="distanceTest")
@Disabled
public class DistanceTest extends LinearOpMode {
    private Rev2mDistanceSensor distance;



    @Override
    public void runOpMode(){
        innit();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }

    public void innit(){
        distance = hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
    }
}
