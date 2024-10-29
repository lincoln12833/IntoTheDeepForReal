package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MM_Pivot {
    private DcMotorEx pivot = null;

    private Rev2mDistanceSensor topLimit;

    private  Rev2mDistanceSensor bottomLimit;

    private final MM_OpMode opMode;

    MM_Pivot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void controlPivot(){

    }

    public void init(){
        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");

        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setTargetPosition(0);

        topLimit = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "topLimit");
        bottomLimit = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "bottomLimit");
    }

}
