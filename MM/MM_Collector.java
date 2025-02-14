package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.COLLECT_BASE_POWER;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.COLLECT_POWER_EFFECTOR;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.COLLECT_CONSTANTS.SCORE_POWER;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private ColorRangeSensor innerSampleSensor;
    private ColorRangeSensor outerSampleSensor;
    private DcMotor wheels;

    public static ElapsedTime collectTime = new ElapsedTime();

    public boolean collectTimeIsStarted = false;
    public double collectPower = COLLECT_BASE_POWER;

    //private boolean toggle = false;

    public static boolean haveSample;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void controlCollector(){
        updateCollectPower();
        if(opMode.gamepad2.right_bumper){
            if (innerSampleSensor.getDistance(DistanceUnit.MM) > 60) {
                wheels.setPower(.6);
                haveSample = false;
            } else if(opMode.robot.transport.pivot.pivot.getCurrentPosition() >= (opMode.robot.transport.pivot.MAX_TICKS *.75) || opMode.gamepad2.a){
                wheels.setPower(SCORE_POWER);
                haveSample = false;
            } else {
                wheels.setPower(0);
                haveSample = true;
            }
        } else if(opMode.gamepad2.left_bumper){
            wheels.setPower(-SCORE_POWER);
        } else {
            wheels.setPower(0);
        }

//        if(currentGamepad2.a && !previousGamepad2.a){
//            toggle = !toggle;
//        }
    }

    public void getSensorStuff(){
        opMode.multipleTelemetry.addData("distance(mm)", innerSampleSensor.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("light detected", innerSampleSensor.getLightDetected());
        opMode.multipleTelemetry.addData("argb", innerSampleSensor.argb());
        opMode.multipleTelemetry.addData("gain", innerSampleSensor.getGain());
        opMode.multipleTelemetry.addData("color", innerSampleSensor.getNormalizedColors().toColor());
        opMode.multipleTelemetry.addData("Sensor status", innerSampleSensor.status());
        opMode.multipleTelemetry.addData("red", innerSampleSensor.red());
        opMode.multipleTelemetry.addData("green", innerSampleSensor.green());
        opMode.multipleTelemetry.addData("blue", innerSampleSensor.blue());
        opMode.multipleTelemetry.update();
    }

    public double getPower(){
        return wheels.getPower();
    }

    public void setPower(double power){
        wheels.setPower(power);
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
        if (innerSampleSensor.getDistance(DistanceUnit.MM) < 60) {
            haveSample = true;
        } else {
            haveSample = false;
        }
        return haveSample;
    }

    public void updateCollectPower(){
        if(outerSampleSensor.getDistance(DistanceUnit.MM) <= 60){
            collectPower = COLLECT_BASE_POWER * COLLECT_POWER_EFFECTOR;
        } else {
            collectPower = COLLECT_BASE_POWER;
        }
    }

    public boolean collectDone(boolean collect, double targetPivotAngle){
        updateCollectPower();
        if (!opMode.robot.drivetrain.collectDone && collect) {
            if (opMode.robot.transport.pivot.getCurrentAngle() < targetPivotAngle + 10 && getPower() == 0) {
                wheels.setPower(collectPower); //previously -.35
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

        innerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");
        outerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "outerSampleLimit");

    }
}
