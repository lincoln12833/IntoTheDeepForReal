package org.firstinspires.ftc.teamcode.MM.MM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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

    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode(){
        alliance = RED;
        goal = BASKET;
        multipleTelemetry.addData("Status", "Initializing... please wait.");
        multipleTelemetry.update();

        initMM();

        while(opModeInInit()){
            robot.navigation.updatePosition(true);
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            multipleTelemetry.addData("Status", "Initialized.");
            if(getClass() == MM_Autos.class){
                multipleTelemetry.addLine("Press right bumper to switch alliance.");
                multipleTelemetry.addLine("Press left bumper to switch goal.");
                multipleTelemetry.addData("Alliance", alliance);
                multipleTelemetry.addData("Goal", goal);

                if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                    alliance = alliance.equals(BLUE)? RED: BLUE;
                }
                if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                    goal = goal.equals(CHAMBER)? BASKET: CHAMBER;
                }
            }
            multipleTelemetry.update();
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
