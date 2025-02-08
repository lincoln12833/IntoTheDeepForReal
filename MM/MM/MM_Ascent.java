package org.firstinspires.ftc.teamcode.MM.MM;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.ASCENT_CONSTANTS.ASCENT_LVL_1;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.ASCENT_CONSTANTS.FOLD_POSITION;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.ASCENT_CONSTANTS.TICK_INCREMENT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Ascent {
    private MM_OpMode opMode;

    ElapsedTime liftTimer = new ElapsedTime();

    private DcMotorEx liftMotor;

    private Servo parkServo;
    public Servo liftRight;
    public Servo liftLeft;

    public boolean lifting = false;
    public boolean pivotInitialPosSet = false;
    public boolean armsMovingUp = false;
    public boolean pivotReadyForLift = false;
    public boolean autoSettingTicks = false;
    public boolean gettingPivotReady = false;
    public int targetPos = 0;

    public MM_Ascent(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void ascendFirstLevel(){
        parkServo.setPosition(ASCENT_LVL_1);
    }
    public void prepareFirstLvlAscent(){
        parkServo.setPosition(ASCENT_LVL_1 + .3);
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
            if(opMode.robot.transport.pivot.getCurrentAngle() >=70 && !armsMovingUp) {
                liftTimer.reset();
                liftLeft.setPosition(.57);
                liftRight.setPosition(.43);
                armsMovingUp = true;
            }

            if(liftTimer.milliseconds() > 1000 && armsMovingUp){
                if(Math.abs(liftMotor.getCurrentPosition()) >= 450) {
                    opMode.robot.transport.pivot.setAngle(40);
                    if (opMode.robot.transport.pivot.getCurrentAngle() <= 45) {
                        pivotReadyForLift = true;
                    }
                }
                liftLeft.getController().pwmDisable();
                liftRight.getController().pwmDisable();
            }
            if(armsMovingUp && liftTimer.milliseconds() > 1200) {
                if(!autoSettingTicks && pivotReadyForLift){
                    autoSettingTicks = true;
                    targetPos = -7250;
                } else if(!gettingPivotReady){
                    gettingPivotReady = true;
                    targetPos = -500;
                }

                if(autoSettingTicks) {
                    if(opMode.gamepad1.right_trigger > 0) {
                        targetPos = (int) (liftMotor.getCurrentPosition() + (opMode.gamepad1.right_trigger * TICK_INCREMENT));
                    } else if (opMode.gamepad1.left_trigger > 0) {
                        targetPos = Math.max((int) (liftMotor.getCurrentPosition() - (opMode.gamepad1.left_trigger * TICK_INCREMENT)), -7300);
                    }
                }
                if (liftMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
                    liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    liftMotor.setTargetPosition(0);
                    liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                }

                liftMotor.setTargetPosition(targetPos);
            }
            opMode.multipleTelemetry.addData("lift ticks", liftMotor.getCurrentPosition());
            opMode.multipleTelemetry.update();
        }
    }

    public void fixString(){
        liftLeft.getController().pwmDisable();
        liftRight.getController().pwmDisable();

        if(opMode.gamepad1.right_trigger > 0) {
            targetPos = (int) (liftMotor.getCurrentPosition() + (opMode.gamepad1.right_trigger * TICK_INCREMENT));
        } else if (opMode.gamepad1.left_trigger > 0) {
            targetPos = Math.max((int) (liftMotor.getCurrentPosition() - (opMode.gamepad1.left_trigger * TICK_INCREMENT)), -7300);
        }
        if (liftMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }

        liftMotor.setTargetPosition(targetPos);
    }

    public void testServos(int operation) {
        if (operation == 1){
            liftRight.getController().pwmDisable();
            liftLeft.getController().pwmEnable();
            liftLeft.setPosition(.8);
        } else if (operation == 2){

            liftRight.setPosition(.4);
        } else if (operation == 4){
            liftRight.getController().pwmEnable();
            liftLeft.getController().pwmEnable();
            liftRight.setPosition(.4);
            liftLeft.setPosition(.6);
        }
    }

    public void testMotor(){
            liftMotor.setPower(.3);
            opMode.sleep(500);
            liftMotor.setPower(-.3);

            opMode.sleep(500);
            liftMotor.setPower(0);
    }

    public void init(){
        parkServo = opMode.hardwareMap.get(Servo.class, "ascentServo");
        liftRight = opMode.hardwareMap.get(Servo.class, "liftRight");
        liftLeft = opMode.hardwareMap.get(Servo.class, "liftLeft");

        liftRight.setPosition(.87);
        liftLeft.setPosition(.13);


        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");

        if(opMode.getClass() == MM_Autos.class) {
            parkServo.setPosition(FOLD_POSITION);
        }

        if(opMode.getClass() == MM_FIX_SERVOS.class) {
            liftLeft.setPosition(.5);
            liftRight.setPosition(.5);
        }
    }
}
