package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private ColorRangeSensor sampleTest;
    private DcMotor wheels;

    public static ElapsedTime collectTime = new ElapsedTime();

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
        collectTime.reset();
        while( opMode.opModeIsActive() && (sampleTest.getDistance(DistanceUnit.MM) < 60 || collectTime.milliseconds() < 1000)) {
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
        if (!opMode.robot.drivetrain.collectDone && collect) {
            if (opMode.robot.transport.pivot.getCurrentAngle() < opMode.robot.transport.pivot.getTargetAngle() + 10 && getPower() == 0 && collect) {
                wheels.setPower(-.6);
                collectTime.reset();
            }
            if (wheels.getPower() < 0 && (haveSample() || !collect || collectTime.milliseconds() > 3000)) {
                wheels.setPower(0);
                return true;
            } else {
                return false;
            }
        }
        return true;
    }

    public void init(){
        wheels = opMode.hardwareMap.get(DcMotor.class, "wheels");

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sampleTest = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");

    }
}
