package org.firstinspires.ftc.teamcode.MM;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();



    @Override
    public void runOpMode(){
        initMM();

        while(opModeInInit()){  //TODO init options

        }

        runProcedures();
    }

    public void runProcedures(){}

    public void initMM(){
        robot = new MM_Robot(this);
        robot.init();
    }
}
