package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.COLLECT_POWER;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.SCORE_POWER;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private ColorRangeSensor sampleTest;
    private DcMotor wheels;

    public static ElapsedTime collectTime = new ElapsedTime();

    public boolean collectTimeIsStarted = false;

    //private boolean toggle = false;

    public static boolean haveSample;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void controlCollector(){
        if(opMode.gamepad2.right_bumper){
            if (sampleTest.getDistance(DistanceUnit.MM) > 60) {
                wheels.setPower(COLLECT_POWER);
                haveSample = false;
            } else if(opMode.robot.transport.pivot.pivot.getCurrentPosition() >= (opMode.robot.transport.pivot.MAX_TICKS *.75) || opMode.gamepad2.a){
                wheels.setPower(SCORE_POWER);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }
        } else if(opMode.gamepad2.left_bumper){
            wheels.setPower(SCORE_POWER);
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
        while( opMode.opModeIsActive() && (sampleTest.getDistance(DistanceUnit.MM) < 60 || collectTime.milliseconds() < 300)) {
            wheels.setPower(SCORE_POWER);
        }
        wheels.setPower(0);
    }

    public void handleCollect(boolean collect) {
        if(collect) {
            if (sampleTest.getDistance(DistanceUnit.MM) > 60) {
                wheels.setPower(COLLECT_POWER);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }
        }
    }

    public boolean haveSample(){
        if (sampleTest.getDistance(DistanceUnit.MM) < 60) {
            haveSample = true;
        } else {
            haveSample = false;
        }
        return haveSample;
    }

    public boolean collectDone(boolean collect, double targetPivotAngle){
        if (!opMode.robot.drivetrain.collectDone && collect) {
            if (opMode.robot.transport.pivot.getCurrentAngle() < targetPivotAngle + 10 && getPower() == 0) {
                wheels.setPower(COLLECT_POWER); //previously -.35
                //collectTime.reset();
            }

            opMode.multipleTelemetry.addData("currentTarget", opMode.robot.transport.pivot.getTargetAngle());
            opMode.multipleTelemetry.addData("finalTarget", targetPivotAngle);
            opMode.multipleTelemetry.addData("currentPivotAngle", opMode.robot.transport.pivot.getCurrentAngle());

            if (!opMode.robot.transport.pivot.pivot.isBusy() && opMode.robot.transport.pivot.getTargetAngle() <= targetPivotAngle){
                if (!collectTimeIsStarted) {
                    collectTime.reset();
                    collectTimeIsStarted = true;
                }
                if (haveSample() || (collectTime.milliseconds() > 1500 && collectTimeIsStarted)) {
                    wheels.setPower(0);
                    collectTimeIsStarted = false;
                    return true;
                } else {
                    return false;
                }
            }
            return false;
        }
        return true;
    }

    public void init(){
        wheels = opMode.hardwareMap.get(DcMotor.class, "wheels");

        wheels.setDirection(DcMotor.Direction.REVERSE);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sampleTest = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");

    }
}
