package org.firstinspires.ftc.teamcode.MM.MM;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.ASCENT_CONSTANTS.ASCENT_LVL_1;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.ASCENT_CONSTANTS.FOLD_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Ascent {
    private MM_OpMode opMode;

    private Servo ascentServo;

    public MM_Ascent(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void ascendFirstLevel(){
        ascentServo.setPosition(ASCENT_LVL_1);
    }

    public void controlAscent(){
        if(opMode.getRuntime() >= 110 && MM_OpMode.currentGamepad1.y && !MM_OpMode.previousGamepad1.y){
            ascentServo.setPosition(ASCENT_LVL_1);
        }

        if(opMode.gamepad1.b){
            ascentServo.setPosition(FOLD_POSITION);
        }
    }

    public void init(){
        ascentServo = opMode.hardwareMap.get(Servo.class, "ascentServo");

        if(opMode.getClass() == MM_Autos.class) {
            ascentServo.setPosition(FOLD_POSITION);
        }
    }
}
