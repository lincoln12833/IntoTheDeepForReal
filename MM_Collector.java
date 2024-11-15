package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private ColorRangeSensor sampleTest;
    private CRServo wheels;

    public static boolean haveSample;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void handleCollect(boolean collect) {
        if(collect) {
            if (sampleTest.getDistance(DistanceUnit.MM) > 30) {
                wheels.setPower(-1);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }
        }
    }

    public void init(){
        wheels = opMode.hardwareMap.get(CRServo.class, "wheels");

        sampleTest = opMode.hardwareMap.get(ColorRangeSensor.class, "limit");

    }
}
