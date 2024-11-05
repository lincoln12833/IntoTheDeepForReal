package org.firstinspires.ftc.teamcode.IntoTheDeepForReal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MM_Slide {

    private final MM_OpMode opMode;

    private DcMotor slide;
    private TouchSensor bottomLimit;

    private final int UPPER_LIMIT = 4000; //FIND THE ACTUAL VAL
    private final int SLIDE_TICK_INCREMENT = 25; //FIND THE ACTUAL VAL

    private boolean isBottomLimitHandled = false;
    private int slideTargetTicks = 0;

    MM_Slide(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void runSlide() {
        opMode.telemetry.addData("target ticks", slideTargetTicks);
        opMode.telemetry.addData("current slide ticks", slide.getCurrentPosition());
        opMode.telemetry.addData("bottom limit is pressed?", bottomLimit.isPressed());
        opMode.telemetry.addData("bottom limit is handled?", isBottomLimitHandled);
        opMode.telemetry.update();

        if (bottomLimit.isPressed() && !isBottomLimitHandled) { // chunk 1
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setPower(0);
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideTargetTicks = 0;
            isBottomLimitHandled = true;

            opMode.telemetry.addData("got into chunk 1!", "");
            opMode.telemetry.update();
        }

        if (opMode.gamepad2.right_trigger > 0.1 || opMode.gamepad2.left_trigger > 0.1) { // chunk 2
            opMode.telemetry.addData("got into chunk 2!", "");
            opMode.telemetry.update();

            if (opMode.gamepad2.left_trigger > 0.1) { // chunk 3
                slideTargetTicks = Math.max(slideTargetTicks - SLIDE_TICK_INCREMENT, 0);

                opMode.telemetry.addData("got into chunk 3!", "");
                opMode.telemetry.update();
            } else if (opMode.gamepad2.right_trigger > 0.1) { // chunk 4
                slideTargetTicks = Math.min(slideTargetTicks + SLIDE_TICK_INCREMENT, UPPER_LIMIT);

                opMode.telemetry.addData("got into chunk 4!", "");
                opMode.telemetry.update();
            }

            slide.setTargetPosition(slideTargetTicks);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            if (!bottomLimit.isPressed()) { // chunk 5
                isBottomLimitHandled = false;

                opMode.telemetry.addData("got into chunk 5!", "");
                opMode.telemetry.update();
            }
        }
    }

    private void init() {
        slide = opMode.hardwareMap.get(DcMotor.class, "slide");
        bottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");

        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
