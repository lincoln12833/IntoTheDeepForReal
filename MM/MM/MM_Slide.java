package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MM_Slide {
    private final MM_OpMode opMode;

    private DcMotor slide;
    private TouchSensor bottomLimit;

    public final int MAX_TICKS = 3000;
    private final int SLIDE_TICK_INCREMENT = 230; //TODO FIND THE ACTUAL VAL
    private final double PULLEY_DIAMETER = 1.503937;
    private final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER;
    private final double TICKS_PER_REV = 537.7;
    private final double TICKS_PER_INCH = (TICKS_PER_REV / PULLEY_CIRCUMFERENCE);
    private final double MAX_EXTENSION_AT_HORIZONTAL = 18;
    private final double MAX_INCHES_BELOW_HORIZONTAL = 4;

    private int maxSlideTicks = 0;
    private boolean BottomLimitIsHandled = false;
    private int slideTargetTicks = 0;
    private boolean homing = false;
    private boolean holdingHome = false;

    MM_Slide(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void runSlide() {
        setMaxSlideTicks(MM_Transport.pivotAngle);

        if ((bottomLimit.isPressed() || holdingHome) && opMode.gamepad2.right_trigger < 0.05){  // dead weight - is this ok, or does it need to be powered?
            if (!holdingHome) { // chunk 1
                slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                slide.setTargetPosition(0);
                slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//                slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  // comment out for powered home
                slide.setPower(0.001);  // comment out for powered home
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // added for powered home
                slideTargetTicks = 0;
                homing = false;
                holdingHome = true;
            }
        } else {
            if (opMode.gamepad2.left_trigger > 0.05) { // coming in
                homing = false;
                //slideTargetTicks = Math.max(slide.getCurrentPosition() - (int)(opMode.gamepad2.left_trigger * SLIDE_TICK_INCREMENT),0);
                slideTargetTicks = slide.getCurrentPosition() - (int)(opMode.gamepad2.left_trigger * SLIDE_TICK_INCREMENT);
            } else if (opMode.gamepad2.right_trigger > 0.05) { // going out
                homing = false;
                //slideTargetTicks = Math.min(slide.getCurrentPosition() + (int)(opMode.gamepad2.right_trigger * SLIDE_TICK_INCREMENT), maxSlideTicks);
                slideTargetTicks = slide.getCurrentPosition() + (int)(opMode.gamepad2.right_trigger * SLIDE_TICK_INCREMENT);
            } else if (opMode.gamepad2.x){ // home
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.setPower(-1);
                slideTargetTicks = 0;
                homing = true;
            }

            slideTargetTicks = Math.min(slideTargetTicks, MM_Transport.maxSlideTicksForAngle);
            slide.setTargetPosition(slideTargetTicks);

            if (holdingHome || !homing) {
                slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }
            holdingHome = false;
        }

        opMode.telemetry.addData("Max slide Ticks", maxSlideTicks);
        opMode.telemetry.addData("target slide ticks - variable", slideTargetTicks);
        opMode.telemetry.addData("current slide ticks", slide.getCurrentPosition());
        opMode.telemetry.addData("bottom limit is pressed?", bottomLimit.isPressed());
        opMode.telemetry.addData("bottom limit is handled?", BottomLimitIsHandled);
        opMode.telemetry.addData("slide power", slide.getPower());
    }

    public void setTargetInches(double targetInches){
        maxSlideTicks = Math.min((int)((42 / Math.cos(Math.toRadians(MM_Transport.pivotAngle))) * TICKS_PER_INCH), MAX_TICKS);
        slide.setTargetPosition((int)(Math.min((targetInches * TICKS_PER_INCH), maxSlideTicks)));
    }
    public void setTargetTicks(int targetTicks){
        maxSlideTicks = Math.min((int)((42 / Math.cos(Math.toRadians(MM_Transport.pivotAngle))) * TICKS_PER_INCH), MAX_TICKS);

        slide.setTargetPosition(Math.min(targetTicks, maxSlideTicks));
    }

    public void home(){
        slide.setTargetPosition(0);
    }

    public boolean slideMovementDone(){
        return !slide.isBusy();
    }

    public void updateSlide(boolean wantMax, double inches){


        maxSlideTicks = Math.min((int)((42 / Math.cos(Math.toRadians(MM_Transport.pivotAngle))) * TICKS_PER_INCH), MAX_TICKS);
        if(wantMax) {
            slide.setTargetPosition(maxSlideTicks);
        } else {
            slide.setTargetPosition((int)(Math.min((inches * TICKS_PER_INCH), Math.min(MAX_TICKS, maxSlideTicks))));
        }
    }

    public void setPower(double power){
        slide.setPower(power);
    }

    private void setMaxSlideTicks(double pivotAngle){
        maxSlideTicks = (int) Math.min(MAX_TICKS, ((MAX_EXTENSION_AT_HORIZONTAL / Math.abs(Math.cos(Math.toRadians(pivotAngle)))) * TICKS_PER_INCH));
        if (pivotAngle < 0) {
            //maxSlideTicks = (int) Math.min((MAX_INCHES_BELOW_HORIZONTAL/ Math.abs(Math.sin(Math.toRadians(pivotAngle))) - 14) * TICKS_PER_INCH,maxSlideTicks);
        }
        MM_Transport.maxSlideTicksForAngle = maxSlideTicks;
    }

    public int getCurrentSlideTicks(){
        return slide.getCurrentPosition();
    }

    private void init() {
        slide = opMode.hardwareMap.get(DcMotor.class, "slide");
        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
