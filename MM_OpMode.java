package org.firstinspires.ftc.teamcode.MM.MM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;
    public static int BLUE = -1;
    public static int RED = 1;
    public static int alliance;

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
        alliance = 1;
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
                multipleTelemetry.addData("Alliance", alliance == 1? "Red": "Blue");
                multipleTelemetry.addData("Goal", goal);

                if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                    alliance *= -1;
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
