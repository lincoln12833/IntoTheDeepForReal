package org.firstinspires.ftc.teamcode.MM;

import static org.firstinspires.ftc.teamcode.MM.MM_Collector.haveSample;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Pivot {
    private final MM_OpMode opMode;

    private DcMotorEx pivot = null;

    private TouchSensor bottomLimit;

    private boolean homing = false;

    public final int TICK_INCREMENT = 28;

    private final int MAX_HEIGHT = 2000;;


    public int targetPos = 0;



    MM_Pivot(MM_OpMode opMode){
        this.opMode = opMode;
        init();

    }

    public void controlPivot(){
        opMode.telemetry.addData("Current pos", pivot.getCurrentPosition());
        opMode.telemetry.addData("target pos", pivot.getTargetPosition());
        opMode.telemetry.addData("Current current", pivot.getCurrent(CurrentUnit.AMPS));
        opMode.telemetry.addData("Current velocity", pivot.getVelocity());
        opMode.telemetry.addData("PID Coefficients", pivot.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        opMode.telemetry.addData("target position tolerance", pivot.getTargetPositionTolerance());
        opMode.telemetry.addData("current alert", pivot.getCurrentAlert(CurrentUnit.AMPS));
        opMode.telemetry.addData("is over current =",  pivot.isOverCurrent());

        if (opMode.gamepad1.x && !bottomLimit.isPressed()){
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(-.7);
            homing = true;
        }

        if (opMode.gamepad1.y){
            homing = false;
            targetPos = MAX_HEIGHT;
        }

        if(bottomLimit.isPressed() || Math.abs(opMode.gamepad1.left_stick_y) > 0.1){
            homing = false;
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(1);
        }

        if (-opMode.gamepad1.left_stick_y > .1){
            targetPos = Math.min(targetPos + TICK_INCREMENT, MAX_HEIGHT);
        } else if (-opMode.gamepad1.left_stick_y < -.1){
            targetPos = Math.max(targetPos - TICK_INCREMENT, 0);
        }

        if (!homing) {
            pivot.setTargetPosition(targetPos);
        }


    }

    public void calculateAngle(){
        MM_Transport.angle = (pivot.getCurrentPosition() / 6.0) * 360 * 537.7;
    }

    public void init(){
        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");

        pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivot.setTargetPosition(0);
        pivot.setPower(1);



        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");
    }

}
