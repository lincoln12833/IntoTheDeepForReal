package org.firstinspires.ftc.teamcode.MM.MM;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.ASCENT_CONSTANTS.ASCENT_LVL_1;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.ASCENT_CONSTANTS.FOLD_POSITION;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Ascent {
    private MM_OpMode opMode;

    ElapsedTime liftTimer = new ElapsedTime();

    private DcMotorEx liftMotor;

    private Servo parkServo;
    private Servo liftRight;
    private Servo liftLeft;

    public boolean lifting = false;
    public boolean pivotInitialPosSet = false;
    public boolean armsMovingUp = false;
    public boolean pivotReadyForLift = false;

    public MM_Ascent(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void ascendFirstLevel(){
        parkServo.setPosition(ASCENT_LVL_1);
    }

    public void controlAscent(){
        if(MM_OpMode.currentGamepad1.y && !MM_OpMode.previousGamepad1.y){
            parkServo.setPosition(ASCENT_LVL_1);
        }

        if(opMode.gamepad1.b){
            parkServo.setPosition(FOLD_POSITION);
        }

        if(opMode.getClass() == MM_TeleOp.class && (opMode.gamepad1.dpad_up || lifting)){
            lifting = true;
            if(!pivotInitialPosSet) {
                opMode.robot.transport.pivot.setAngle(90);
                pivotInitialPosSet = true;
            }
            if(opMode.robot.transport.pivot.getCurrentAngle() >=70) {
                liftTimer.reset();
                liftLeft.setPosition(1);
                liftRight.setPosition(1);
                armsMovingUp = true;
            }

            if(liftTimer.milliseconds() > 500 && armsMovingUp){
                opMode.robot.transport.pivot.setAngle(50);
                if(opMode.robot.transport.pivot.getCurrentAngle() <= 57){
                    pivotReadyForLift = true;
                }
                if(pivotReadyForLift) {
                    liftLeft.getController().pwmDisable();
                    liftRight.getController().pwmDisable();
                    liftMotor.setPower(-.3);
                }
            }
        }
    }

    public void testServos(int operation) {
        if (operation == 1){
            liftLeft.getController().pwmEnable();
            liftLeft.setPosition(1);
        } else if (operation == 2){
            liftRight.getController().pwmEnable();
            liftRight.setPosition(1);
        }
    }

    public void testMotor(){
            liftMotor.setPower(.3);
            opMode.sleep(500);
            liftMotor.setPower(-.3);

            opMode.sleep(500);
    }

    public void init(){
        parkServo = opMode.hardwareMap.get(Servo.class, "ascentServo");
        liftRight = opMode.hardwareMap.get(Servo.class, "liftRight");
        liftLeft = opMode.hardwareMap.get(Servo.class, "liftLeft");

        liftLeft.getController().pwmDisable();
        liftRight.getController().pwmDisable();

        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");

        if(opMode.getClass() == MM_Autos.class) {
            parkServo.setPosition(FOLD_POSITION);
        }
    }
}
