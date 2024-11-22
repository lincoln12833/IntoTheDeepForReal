package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MM_Slide {

    private final MM_OpMode opMode;

    private DcMotor slide;
    private TouchSensor bottomLimit;

    private final int UPPER_LIMIT = 4000; //TODO FIND THE ACTUAL VAL
    private final int SLIDE_TICK_INCREMENT = 45; //TODO FIND THE ACTUAL VAL

    private final double PULLEY_DIAMETER = 1.503937;
    private final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER;
    private final double TICKS_PER_REV = 537.7;
    private final double TICKS_PER_INCH = (TICKS_PER_REV / PULLEY_CIRCUMFERENCE);

    private int maxSlideTicks = 0;

    private boolean isBottomLimitHandled = false;
    private int slideTargetTicks = 0;


    MM_Slide(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void runSlide() {
        maxSlideTicks = (int) Math.min(UPPER_LIMIT, ((42 / Math.cos(Math.toRadians(MM_Transport.pivotAngle))) * TICKS_PER_INCH));

        if (bottomLimitIsTriggered() && !isBottomLimitHandled) { // chunk 1
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setPower(0);
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideTargetTicks = 0;
            isBottomLimitHandled = true;
        }

        if (opMode.gamepad2.right_trigger > 0.1 || opMode.gamepad2.left_trigger > 0.1) { // chunk 2

            if (opMode.gamepad2.left_trigger > 0.1) { // chunk 3
                slideTargetTicks = Math.max(slideTargetTicks - SLIDE_TICK_INCREMENT, 0);

            } else if (opMode.gamepad2.right_trigger > 0.1) { // chunk 4
                slideTargetTicks = Math.min(slideTargetTicks + SLIDE_TICK_INCREMENT, maxSlideTicks);
            }

            slide.setTargetPosition(slideTargetTicks);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            if (!bottomLimitIsTriggered()) { // chunk 5
                isBottomLimitHandled = false;
            }
        }

        opMode.telemetry.addData("Max slide Ticks", maxSlideTicks);
        opMode.telemetry.addData("target slide ticks", slideTargetTicks);
        opMode.telemetry.addData("current slide ticks", slide.getCurrentPosition());
        opMode.telemetry.addData("bottom limit is pressed?", bottomLimit.isPressed());
        opMode.telemetry.addData("bottom limit is handled?", isBottomLimitHandled);
    }

    public void home(){
        slide.setTargetPosition(0);
    }

    public void updateSlide(boolean wantMax, double inches){
        maxSlideTicks = Math.min((int)((42 / Math.cos(Math.toRadians(MM_Transport.pivotAngle))) * TICKS_PER_INCH), UPPER_LIMIT);
        if(wantMax) {
            slide.setTargetPosition(maxSlideTicks);
        } else {
            slide.setTargetPosition((int)(Math.min((inches * TICKS_PER_INCH), Math.min(UPPER_LIMIT, maxSlideTicks))));
        }

    }

    public boolean bottomLimitIsTriggered(){
        return !bottomLimit.isPressed();
    }

    private void init() {
        slide = opMode.hardwareMap.get(DcMotor.class, "slide");
        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");

        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
