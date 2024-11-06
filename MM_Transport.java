package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MM_Transport {
    private final MM_OpMode opMode;

    public MM_Slide slide;
    public MM_Pivot pivot;

    MM_Transport(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }


    public void init() {
        pivot = new MM_Pivot(opMode);
        slide = new MM_Slide(opMode);
    }
}
