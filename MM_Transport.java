package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MM_Transport {
    private final MM_OpMode opMode;

    public MM_Slide slide;
    public MM_Pivot pivot;

    public static double angle = 0;

    MM_Transport(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void chamberPos(){

    }

    public void runTransport(){
        pivot.controlPivot();
        slide.runSlide();
    }
    public void init() {
        pivot = new MM_Pivot(opMode);
        slide = new MM_Slide(opMode);
    }
}
