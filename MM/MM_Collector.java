package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.currentGamepad2;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.previousGamepad2;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private ColorRangeSensor sampleTest;
    private DcMotor wheels;

    //private boolean toggle = false;

    public static boolean haveSample;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void controlCollector(){
        if(opMode.gamepad2.right_bumper){
            if (sampleTest.getDistance(DistanceUnit.MM) > 60) {
                wheels.setPower(-.6);
                haveSample = false;
            } else if(opMode.robot.transport.pivot.pivot.getCurrentPosition() >= (opMode.robot.transport.pivot.MAX_TICKS *.75) || opMode.gamepad2.a){
                wheels.setPower(-1);
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

//        if(currentGamepad2.a && !previousGamepad2.a){
//            toggle = !toggle;
//        }
    }

    public double getPower(){
        return wheels.getPower();
    }

    public void setPower(double power){
        wheels.setPower(power);
    }

    public void score(){
        while(sampleTest.getDistance(DistanceUnit.MM) < 60) {
            wheels.setPower(-1);
        }
        wheels.setPower(0);
    }

    public void handleCollect(boolean collect) {
        if(collect) {
            if (sampleTest.getDistance(DistanceUnit.MM) > 60) {
                wheels.setPower(-.6);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }

        }

    }

    public boolean haveSample(){
        if (sampleTest.getDistance(DistanceUnit.MM) > 60) {
            haveSample = false;
        } else {
            haveSample = true;
        }
        return haveSample;
    }

    public boolean collectDone(boolean collect){
        if(haveSample() || !collect || MM_Drivetrain.collectTime.milliseconds() > 1000){
            wheels.setPower(0);
            return true;
        } else{
            return false;
        }
    }

    public void init(){
        wheels = opMode.hardwareMap.get(DcMotor.class, "wheels");

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sampleTest = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");

    }
}
