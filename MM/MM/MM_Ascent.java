package org.firstinspires.ftc.teamcode.MM.MM;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Ascent {
    private MM_OpMode opMode;

    private Servo ascentServo;

    @Config
    public static class ASCENT_CONSTANTS {
        public static double ASCENT_LVL_1 = .7;
    }

    public MM_Ascent(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void ascendFirstLevel(){
        ascentServo.setPosition(ASCENT_CONSTANTS.ASCENT_LVL_1);
    }

    public void controlAscent(){
        if(opMode.getRuntime() >= 110 && MM_OpMode.currentGamepad1.y && !MM_OpMode.previousGamepad1.y){
            ascentServo.setPosition(ASCENT_CONSTANTS.ASCENT_LVL_1);
        }
    }

    public void init(){
        ascentServo = opMode.hardwareMap.get(Servo.class, "ascentServo");

        if(opMode.getClass() == MM_Autos.class) {
            ascentServo.setPosition(0);
        }
    }
}
