package org.firstinspires.ftc.teamcode.MM;

import static org.firstinspires.ftc.teamcode.MM.MM_Collector.haveSample;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Pivot {
    private final MM_OpMode opMode;

    private DcMotorEx pivot = null;

    private TouchSensor bottomLimit;

    private boolean homing = false;

    public final int TICK_INCREMENT = 28;

    private final int MAX_HEIGHT = 1400;;

    private final double DEGREE_OFFSET = 37;


    public int targetPos = 0;

    private boolean homingHandled = false;



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
        if (opMode.gamepad2.x && !bottomLimitIsTriggered()){

            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(-.7);
            homing = true;
        }

        if (bottomLimitIsTriggered()){
            targetPos = 0;
            pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(0);
            homing = false;

        } else if (!homing) {
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(1);
        }

        if (opMode.gamepad2.y){
            homing = false;
            targetPos = MAX_HEIGHT;
        }

        if(Math.abs(opMode.gamepad2.left_stick_y) > 0.1 && homing){
            homing = false;
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(1);
            targetPos = pivot.getCurrentPosition();
        }

        if (-opMode.gamepad2.left_stick_y > .1){
            if (pivot.getPower() < 1){
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(1);
            }
            targetPos = Math.min(targetPos + TICK_INCREMENT, MAX_HEIGHT);
        } else if (-opMode.gamepad2.left_stick_y < -.1){
            targetPos = Math.max(targetPos - TICK_INCREMENT, 0);
        }

        pivot.setTargetPosition(targetPos);

    }

    public boolean bottomLimitIsTriggered(){
        return bottomLimit.isPressed();
    }

    public void updatePivot(double targetPivotAngle){
        pivot.setTargetPosition((int)((targetPivotAngle * 6) / 360 / 537.7));
        calculateAngle();
    }

    public void calculateAngle(){
        MM_Transport.angle = ((pivot.getCurrentPosition() / 6.0) * 360 * 537.7) + DEGREE_OFFSET;
    }

    public void home(){
        pivot.setTargetPosition(0);
    }

    public void setAngle(double angle){
        pivot.setTargetPosition((int)((angle * 6) / 360 / 537.7));
    }

    public void init(){
        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");

        pivot.setDirection(DcMotorEx.Direction.REVERSE);

        pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivot.setTargetPosition(0);
        pivot.setPower(1);



        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "pivotBottomLimit");
    }

}
