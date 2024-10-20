package org.firstinspires.ftc.teamcode.MM;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;
    private final String BLUE = "Blue";
    private final String RED = "Red";
    private String alliance = BLUE;

    private final String CHAMBER = "Chamber";
    private final String BASKET = "Basket";
    private String goal = BASKET;

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();



    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initializing... please wait.");
        telemetry.update();

        initMM();

        while(opModeInInit()){  //TODO init options
            telemetry.addData("Status", "Initialized.");
            if(getClass() == MM_Autos.class){
                telemetry.addLine("Press right bumper to switch alliance.");
                telemetry.addLine("Press left bumper to switch goal.");
                telemetry.addData("Alliance", alliance);
                telemetry.addData("Goal", goal);

                if(gamepad1.right_bumper){
                    alliance = alliance.equals(BLUE)? RED: BLUE;
                }
                if(gamepad1.left_bumper){
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
    }
}
