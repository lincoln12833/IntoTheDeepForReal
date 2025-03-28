package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.COLLECT_BASE_POWER;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.GRAB_POS;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.SCORE_POWER;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.SPEC_OPEN_POS;

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
    public Servo specClaw;

    public static ElapsedTime collectTime = new ElapsedTime();
    public static ElapsedTime outtakeTimer = new ElapsedTime();

    public boolean collectTimeIsStarted = false;
    public double collectPower = COLLECT_BASE_POWER;

    public static boolean haveSample;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void controlCollector(){
        if(opMode.gamepad2.right_bumper){
            if(opMode.robot.transport.pivot.pivot.getCurrentPosition() >= (opMode.robot.transport.pivot.MAX_TICKS *.75) || opMode.gamepad2.a){
                wheels.setPower(SCORE_POWER);
                haveSample = false;
            } else if ((innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || outerSampleSensor.getDistance(DistanceUnit.MM) < 67.5) && !haveSample) {
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

    }
    public void sensorTelemetry(){
        //debug if here
        opMode.multipleTelemetry.addData("haveSample", haveSample);
        opMode.multipleTelemetry.addData("outer distance(mm)", outerSampleSensor.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("distance(mm)", innerSampleSensor.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("light detected", innerSampleSensor.getLightDetected());
        opMode.multipleTelemetry.addData("color", innerSampleSensor.getNormalizedColors().toColor());
        opMode.multipleTelemetry.addData("red", innerSampleSensor.red());
        opMode.multipleTelemetry.addData("green", innerSampleSensor.green());
        opMode.multipleTelemetry.addData("blue", innerSampleSensor.blue());
    }
    public double getPower(){
        return wheels.getPower();
    }
    public void openSpecClaw(){
        specClaw.setPosition(SPEC_OPEN_POS);
    }
    public void releaseSample(){
        collectTime.reset();
        while( opMode.opModeIsActive() && (haveSample() || collectTime.milliseconds() < 1000)) {
            wheels.setPower(SCORE_POWER);
        }
        wheels.setPower(0);
    }
    public void discardSample(){
        collectTime.reset();
        while( opMode.opModeIsActive() && (innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || collectTime.milliseconds() < 250)) {
            wheels.setPower(-SCORE_POWER);
        }
        wheels.setPower(0);
    }

    public boolean haveSample(){
        haveSample = innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || outerSampleSensor.getDistance(DistanceUnit.MM) < 65;
        opMode.multipleTelemetry.addData("have sample", haveSample);

        return haveSample;
    }

    public void runSpecClaw(){
        if((MM_OpMode.currentGamepad2.right_bumper && !MM_OpMode.previousGamepad2.right_bumper) && opMode.gamepad2.b){
            specClaw.setPosition(specClaw.getPosition() == SPEC_OPEN_POS? GRAB_POS: SPEC_OPEN_POS);
        }
    }

    public void scoreSpec(){
        specClaw.setPosition(SPEC_OPEN_POS);
    }

    public void collectSpec(){
        specClaw.setPosition(GRAB_POS);
    }


    public boolean collectDone(double targetPivotAngle){
        if (!opMode.robot.drivetrain.collectDone) {
            if (opMode.robot.transport.pivot.getCurrentAngle() < targetPivotAngle + 10 && getPower() == 0) {
                wheels.setPower(1); //previously -.35
                //collectTime.reset();
            }
            if (!opMode.robot.transport.pivot.pivot.isBusy() && opMode.robot.transport.pivot.getTargetAngle() == targetPivotAngle){
                if (!collectTimeIsStarted) {
                    collectTime.reset();
                    collectTimeIsStarted = true;
                }

            }
            if (haveSample || haveSample() || (collectTime.milliseconds() > 1500 && collectTimeIsStarted)) {
                wheels.setPower(0);
                collectTimeIsStarted = false;
                return true;
            }
            return false;
        }
        return true;
    }

    public void init(){
        wheels = opMode.hardwareMap.get(DcMotor.class, "wheels");
        specClaw = opMode.hardwareMap.get(Servo.class, "specClaw");
        if(opMode.getClass() == MM_Autos.class) {
            specClaw.setPosition(GRAB_POS);
        } else {
            specClaw.setPosition(SPEC_OPEN_POS);
        }


        wheels.setDirection(DcMotor.Direction.REVERSE);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        innerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");
        outerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "outerSampleLimit");

        opMode.multipleTelemetry.addData("distance(mm)", innerSampleSensor.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("light detected", innerSampleSensor.getLightDetected());
        opMode.multipleTelemetry.update();
    }
}
