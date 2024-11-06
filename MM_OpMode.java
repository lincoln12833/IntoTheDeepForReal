package org.firstinspires.ftc.teamcode.IntoTheDeepForReal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;
    public static String BLUE = "Blue";
    public static String RED = "Red";
    public static String alliance;

    public static String CHAMBER = "Chamber";
    public static String BASKET = "Basket";
    public static String goal;

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();



    @Override
    public void runOpMode(){
        alliance = BLUE;
        goal = CHAMBER;
        telemetry.addData("Status", "Initializing... please wait.");
        telemetry.update();

        initMM();

        while(opModeInInit()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            telemetry.addData("Status", "Initialized.");
            if(getClass() == MM_Autos.class){
                telemetry.addLine("Press right bumper to switch alliance.");
                telemetry.addLine("Press left bumper to switch goal.");
                telemetry.addData("Alliance", alliance);
                telemetry.addData("Goal", goal);

                if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                    alliance = alliance.equals(BLUE)? RED: BLUE;
                }
                if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                    goal = goal.equals(CHAMBER)? BASKET: CHAMBER;
                }
            }
            telemetry.update();
        }

        runProcedures();
    }

    public void runProcedures(){}

    public void initMM(){
        robot = new MM_Robot(this);
        robot.init();

        currentGamepad1.copy(gamepad1);
    }
}
