package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MM_Pivot {
    private final MM_OpMode opMode;

    public DcMotorEx pivot = null;
    private TouchSensor bottomLimit;

    public final double TICKS_PER_REV = 1992.6;
    public final double TICKS_PER_SHAFT_DEGREE = TICKS_PER_REV / 360;
    public final double GEAR_RATIO = 6.0;
    public final double TICKS_PER_PIVOT_DEGREE = TICKS_PER_SHAFT_DEGREE * GEAR_RATIO;
    public final int TICK_INCREMENT = 150;

    private final double OFFSET_PIVOT_ANGLE = -25; //Compensation for negative start angle: 25.7223
    private final double MAX_PIVOT_ANGLE = 90;
    public final int MAX_TICKS = (int)(TICKS_PER_PIVOT_DEGREE * (MAX_PIVOT_ANGLE - OFFSET_PIVOT_ANGLE)); //1450

    private boolean homing = false;
    public int targetPos = 0;
    private boolean modeInPosition = false;

    MM_Pivot(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void controlPivot(){
        if(bottomLimit.isPressed() && !(opMode.gamepad2.left_stick_y < -.1)){
            if(pivot.getCurrentPosition() != 0){
                pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pivot.setPower(0);
                targetPos = 0;
                homing = false;
            }
        } else {
            if (Math.abs(opMode.gamepad2.left_stick_y) > .1) {
                homing = false;
                targetPos = Math.min((int) (pivot.getCurrentPosition() + (-opMode.gamepad2.left_stick_y * TICK_INCREMENT)), MAX_TICKS);
            } else if (opMode.gamepad2.y){
                homing = false;
                targetPos = MAX_TICKS;
            } else if (opMode.gamepad2.x){ // home
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pivot.setPower(-.7);
                targetPos = 0;
                homing = true;
            } else if (homing){
                home();
            }
            pivot.setTargetPosition(targetPos);

            if ( (pivot.getMode() != DcMotor.RunMode.RUN_TO_POSITION && !homing)) { //Only when coming off touch sensor or stopping homing
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setPower(1);
            }
        }

        MM_Transport.pivotAngle = getCurrentAngle();

        opMode.telemetry.addData("angle", MM_Transport.pivotAngle);
        opMode.telemetry.addData("Current pos", pivot.getCurrentPosition()); //telemetry
        opMode.telemetry.addData("target pos", pivot.getTargetPosition());
        opMode.telemetry.addData("Current current", pivot.getCurrent(CurrentUnit.AMPS));
        opMode.telemetry.addData("Current velocity", pivot.getVelocity());
        opMode.telemetry.addData("PID Coefficients", pivot.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        opMode.telemetry.addData("target position tolerance", pivot.getTargetPositionTolerance());
        opMode.telemetry.addData("current alert", pivot.getCurrentAlert(CurrentUnit.AMPS));
        opMode.telemetry.addData("is over current =",  pivot.isOverCurrent());
    }

    public boolean bottomLimitIsTriggered(){
        return bottomLimit.isPressed();
    }

    public void updatePivot(double targetPivotAngle){
        setAngle(targetPivotAngle);
        getCurrentAngle();
    }

    public double getCurrentAngle(){
        return (pivot.getCurrentPosition() / TICKS_PER_PIVOT_DEGREE) + OFFSET_PIVOT_ANGLE;
    }

    public void home(){
        if (opMode.robot.transport.slide.getCurrentSlideTicks() < MM_Transport.maxSlideTicksForAngle) {
            pivot.setTargetPosition(0); // have to wait for slide - autos only
        } else{
            pivot.setTargetPosition(pivot.getCurrentPosition());
        }
    }

    public void setAngle(double angle){
        pivot.setTargetPosition((int)((angle - OFFSET_PIVOT_ANGLE) * TICKS_PER_PIVOT_DEGREE));
    }

    public void init(){
        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");

        pivot.setDirection(DcMotorEx.Direction.REVERSE);

        pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setTargetPosition(0);
        pivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        pivot.setPower(1);

        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "pivotBottomLimit");
    }
}
