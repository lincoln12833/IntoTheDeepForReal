package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.COLLECT_BASE_POWER;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.COLLECT_POWER_EFFECTOR;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.SCORE_POWER;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private ColorRangeSensor innerSampleSensor;
    private ColorRangeSensor outerSampleSensor;
    private DcMotor wheels;
    private Servo specClaw;

    public static ElapsedTime collectTime = new ElapsedTime();
    public static ElapsedTime outtakeTimer = new ElapsedTime();

    public boolean collectTimeIsStarted = false;
    public double collectPower = COLLECT_BASE_POWER;

    //private boolean toggle = false;

    public static boolean haveSample;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void controlCollector(){
        //updateCollectPower();
        if(opMode.gamepad2.right_bumper){
            if(opMode.robot.transport.pivot.pivot.getCurrentPosition() >= (opMode.robot.transport.pivot.MAX_TICKS *.75) || opMode.gamepad2.a){
                wheels.setPower(SCORE_POWER);
                haveSample = false;
            } else if ((innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || outerSampleSensor.getDistance(DistanceUnit.MM) < 70) && !haveSample) {
                wheels.setPower(0);
                haveSample = true;
            } else if (!haveSample) {
                wheels.setPower(collectPower);
            }
        } else if(opMode.gamepad2.left_bumper){
            wheels.setPower(-SCORE_POWER);
            haveSample = false;
        } else if(outtakeTimer.milliseconds() < 125) {
            wheels.setPower(-.3);
        } else if (collectTime.milliseconds() < 125) {
            wheels.setPower(.3);
        } else {
                wheels.setPower(0);

        }

        if ( MM_OpMode.currentGamepad2.dpad_down && !MM_OpMode.previousGamepad2.dpad_down){
        outtakeTimer.reset();
        }
        if(MM_OpMode.currentGamepad2.dpad_up && !MM_OpMode.previousGamepad2.dpad_up){
            collectTime.reset();
        }

//        if(currentGamepad2.a && !previousGamepad2.a){
//            toggle = !toggle;
//        }
    }

    public void getSensorStuff(){
        opMode.multipleTelemetry.addData("outer distance(mm)", outerSampleSensor.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("distance(mm)", innerSampleSensor.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("light detected", innerSampleSensor.getLightDetected());
        opMode.multipleTelemetry.addData("argb", innerSampleSensor.argb());
        opMode.multipleTelemetry.addData("gain", innerSampleSensor.getGain());
        opMode.multipleTelemetry.addData("color", innerSampleSensor.getNormalizedColors().toColor());
        opMode.multipleTelemetry.addData("Sensor status", innerSampleSensor.status());
        opMode.multipleTelemetry.addData("red", innerSampleSensor.red());
        opMode.multipleTelemetry.addData("green", innerSampleSensor.green());
        opMode.multipleTelemetry.addData("blue", innerSampleSensor.blue());
        //opMode.multipleTelemetry.update();
    }

    public double getPower(){
        return wheels.getPower();
    }

    public void setPower(double power){
        wheels.setPower(power);
    }

    public void teleScoreSpec(){
        specClaw.setPosition(.75);
    }

    public void score(){
        collectTime.reset();
        while( opMode.opModeIsActive() && (innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || collectTime.milliseconds() < 500)) {
            wheels.setPower(SCORE_POWER);
        }
        wheels.setPower(0);
    }

    public void uncollect(){
        collectTime.reset();
        while( opMode.opModeIsActive() && (innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || collectTime.milliseconds() < 250)) {
            wheels.setPower(-SCORE_POWER);
        }
        wheels.setPower(0);
    }

    public void handleCollect(boolean collect) {
        if(collect) {
            if (innerSampleSensor.getDistance(DistanceUnit.MM) > 60) {
                wheels.setPower(collectPower);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }
        }
    }

    public boolean haveSample(){
        if (innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || outerSampleSensor.getDistance(DistanceUnit.MM) < 65) {
            haveSample = true;
        } else {
            haveSample = false;
        }
        opMode.multipleTelemetry.addData("have sample", haveSample);

        return haveSample;
    }

    public void runSpecClaw(){
        if((MM_OpMode.currentGamepad2.right_bumper && !MM_OpMode.previousGamepad2.right_bumper) && opMode.gamepad2.b){
            specClaw.setPosition(specClaw.getPosition() == .75? 1: .75);
        }
    }

    public void scoreSpec(){
        specClaw.setPosition(.75);
    }

    public void collectSpec(){
        specClaw.setPosition(1);
    }

    public void updateCollectPower(){
        if(outerSampleSensor.getDistance(DistanceUnit.MM) <= 80){
            collectPower = COLLECT_BASE_POWER * COLLECT_POWER_EFFECTOR;
        } else {
            collectPower = COLLECT_BASE_POWER;
        }
    }

    public boolean collectDone(boolean collect, double targetPivotAngle){
        //updateCollectPower();
        getSensorStuff();
        if (!opMode.robot.drivetrain.collectDone && collect) {
            if (opMode.robot.transport.pivot.getCurrentAngle() < targetPivotAngle + 10 && getPower() == 0) {
                wheels.setPower(1); //previously -.35
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

    public boolean testCollectDone(boolean collect){
        updateCollectPower();
        if (!opMode.robot.drivetrain.collectDone && collect) {
            wheels.setPower(collectPower); //previously -.35

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
        return true;
    }

    public void init(){
        wheels = opMode.hardwareMap.get(DcMotor.class, "wheels");
        specClaw = opMode.hardwareMap.get(Servo.class, "specClaw");
        specClaw.setPosition(1);

        wheels.setDirection(DcMotor.Direction.REVERSE);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        innerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");
        outerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "outerSampleLimit");

        opMode.multipleTelemetry.addData("distance(mm)", innerSampleSensor.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("light detected", innerSampleSensor.getLightDetected());
        opMode.multipleTelemetry.update();
    }
}
