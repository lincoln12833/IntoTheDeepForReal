package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private ColorRangeSensor sampleTest;
    private DcMotor wheels;

    public static boolean haveSample;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void controlCollector(){
        if(opMode.gamepad2.right_bumper){
            if (sampleTest.getDistance(DistanceUnit.MM) > 60 || MM_Transport.pivot.pivot.getCurrentPosition() > 1200) {
                wheels.setPower(-.75);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }
        } else if(opMode.gamepad2.left_bumper){
            wheels.setPower(1);
        } else {
            wheels.setPower(0);
        }
    }


    public void handleCollect(boolean collect) {
        if(collect) {
            if (sampleTest.getDistance(DistanceUnit.MM) > 30) {
                wheels.setPower(-75);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }
        }
    }

    public void init(){
        wheels = opMode.hardwareMap.get(DcMotor.class, "wheels");

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sampleTest = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");

    }
}
