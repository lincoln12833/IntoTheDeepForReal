package org.firstinspires.ftc.teamcode.MM;

import static org.firstinspires.ftc.teamcode.MM.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM.MM_OpMode.previousGamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MM_Slide {

    private final MM_OpMode opMode;

    private DcMotor slide;
    private TouchSensor bottomLimit;

    private final int UPPER_LIMIT = 4000; //TODO FIND THE ACTUAL VAL
    private final int SLIDE_TICK_INCREMENT = 25; //TODO FIND THE ACTUAL VAL

    private final double PULLEY_DIAMETER = 1.503937;
    private final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER;
    private final double TICKS_PER_REV = 537.7;
    private final double TICKS_PER_INCH = (TICKS_PER_REV / PULLEY_CIRCUMFERENCE);

    private double angle = 45;

    private int maxTicks = (int)((42 / (Math.cos(Math.toRadians(angle)))) * TICKS_PER_INCH);

    private boolean isBottomLimitHandled = false;
    private int slideTargetTicks = 0;


    MM_Slide(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void runSlide() {
        opMode.telemetry.addData("target slide ticks", slideTargetTicks);
        opMode.telemetry.addData("current slide ticks", slide.getCurrentPosition());
        opMode.telemetry.addData("bottom limit is pressed?", bottomLimit.isPressed());
        opMode.telemetry.addData("bottom limit is handled?", isBottomLimitHandled);

        if (bottomLimit.isPressed() && !isBottomLimitHandled) { // chunk 1
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setPower(0);
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideTargetTicks = 0;
            isBottomLimitHandled = true;
        }

        if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            angle += 1;
        } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            angle -= 1;
        }

        if (opMode.gamepad2.right_trigger > 0.1 || opMode.gamepad2.left_trigger > 0.1) { // chunk 2

            if (opMode.gamepad2.left_trigger > 0.1) { // chunk 3
                slideTargetTicks = Math.max(slideTargetTicks - SLIDE_TICK_INCREMENT, 0);

            } else if (opMode.gamepad2.right_trigger > 0.1) { // chunk 4
                slideTargetTicks = Math.min(slideTargetTicks + SLIDE_TICK_INCREMENT, Math.min(UPPER_LIMIT, maxTicks));
            }

            slide.setTargetPosition(slideTargetTicks);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            if (!bottomLimit.isPressed()) { // chunk 5
                isBottomLimitHandled = false;
            }
        }
        maxTicks = (int)((42 / Math.cos(Math.toRadians(angle))) * TICKS_PER_INCH);

        opMode.telemetry.addData("Max Ticks", maxTicks);
        opMode.telemetry.addData("angle", angle);
    }

    private void init() {
        slide = opMode.hardwareMap.get(DcMotor.class, "slide");
        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");

        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
