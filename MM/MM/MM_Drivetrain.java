package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.DRIVE_CONSTANTS.DRIVE_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.DRIVE_CONSTANTS.HEADING_ERROR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.CHAMBER;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.previousGamepad1;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.DRIVE_CONSTANTS.TANGENT_THRESHOLD;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Drivetrain {

    private final MM_OpMode opMode;

    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;

    private IMU imu;

    private Rev2mDistanceSensor backDistance;

    public static double MAX_POWER = .5; //previously 1
    public static double SLOW_POWER = .5;

    public static double MAX_TURN_POWER = .5; //previously .5
    public static double MIN_TURN_POWER = .15;
    public static double GYRO_TURN_P_COEFF = .016;




    private final double DISTANCE_THRESHOLD = .37;


    private double xError;
    private double yError;
    private double headingError;

    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;

    private double heading;

    private boolean slow = false;
    private boolean strafeDone = false;
    private boolean driveDone = false;
    private boolean rotateDone = false;
    boolean collectDone = false;

    public static boolean robotAtLocation = false;

    double targetPos;
    double targetDrivePos;
    double targetStrafePos;
    double driveInchesError;
    double distance;
    double distanceError;
    double strafeInchesError;

    MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveWithSticks() {
        double drivePower = -opMode.gamepad1.left_stick_y; //Math.pow(-opMode.gamepad1.left_stick_y, 2) * (opMode.gamepad1.left_stick_y / Math.abs(opMode.gamepad1.left_stick_y)) ;
        double strafePower = opMode.gamepad1.left_stick_x;//Math.pow(opMode.gamepad1.left_stick_x, 2)* (opMode.gamepad1.left_stick_x / Math.abs(opMode.gamepad1.left_stick_x)) ;
        double rotatePower = -opMode.gamepad1.right_stick_x;//Math.pow(-opMode.gamepad1.right_stick_x, 2) * (opMode.gamepad1.right_stick_x / Math.abs(opMode.gamepad1.right_stick_x)); //left is a positive rotation

        if (!opMode.robot.travelling) {
            flPower = drivePower + strafePower - rotatePower;
            frPower = drivePower - strafePower + rotatePower;
            blPower = drivePower - strafePower - rotatePower;
            brPower = drivePower + strafePower + rotatePower;
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            slow = !slow;
        }

        normalize(1);

        if (slow){
            flPower *= SLOW_POWER;
            frPower *= SLOW_POWER;
            blPower *= SLOW_POWER;
            brPower *= SLOW_POWER;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    public boolean driveToPosition(double targetX, double targetY, double maxPower, double fineMaxPower, double targetHeading, double rotateFactor, double fineThreshold, double targetPivotAngle, double targetSlidePos, boolean slideWantMax, boolean collect) {
        collectDone = true;
        robotAtLocation = false;

        calculateAndSetDrivePowers(targetX, targetY, maxPower, targetHeading, rotateFactor);
        while (opMode.opModeIsActive() && !allMovementDone(fineThreshold < 0 && collect, fineThreshold >= 0 && collect?targetPivotAngle + 20: targetPivotAngle, DRIVE_ERROR_THRESHOLD)) {
            if (targetPivotAngle > 80 && !opMode.robot.collector.haveSample() && !MM_OpMode.goal.equals(CHAMBER)){
                break;

            }
            if(Math.abs(opMode.gamepad1.left_stick_x) > 0.1 || Math.abs(opMode.gamepad1.left_stick_y) > 0.1 || Math.abs(opMode.gamepad1.right_stick_y) > 0.1  ){
                opMode.robot.travelling = false;
                opMode.robot.collecting = false;
                opMode.robot.slideTargetSet = false;
                opMode.robot.scoring = false;
                break;
            }
            if (driveDone && strafeDone && rotateDone){
                setDrivePowersToZero();

                if(fineThreshold < 0){
                    break;
                }
            } else{
                calculateAndSetDrivePowers(targetX, targetY, maxPower, targetHeading, rotateFactor);
            }
            opMode.robot.collector.getSensorStuff();
            opMode.multipleTelemetry.addData("Status", "driving to pos");
            opMode.robot.transport.updateTransport(targetPivotAngle, targetSlidePos, slideWantMax, collect);
            opMode.multipleTelemetry.update();
        }
        if(fineThreshold >= 0){
            collectDone = !collect;
            robotAtLocation = false;

            while(opMode.opModeIsActive() && !allMovementDone(collect, targetPivotAngle, fineThreshold)){
                if (targetPivotAngle > 80 && !opMode.robot.collector.haveSample() && !MM_OpMode.goal.equals(CHAMBER)){
                    break;
                }
                if (driveDone && strafeDone && rotateDone){
                    robotAtLocation = true;
                    setDrivePowersToZero();
                } else{
                    robotAtLocation = false;
                    calculateAndSetDrivePowers(targetX, targetY, fineMaxPower, targetHeading, 0.037);
                }
                opMode.robot.transport.updateTransport(targetPivotAngle, targetSlidePos, slideWantMax, collect);
                opMode.robot.collector.getSensorStuff();
                opMode.multipleTelemetry.addData("doneCollect", collectDone);

                opMode.multipleTelemetry.addData("Status", "adjusting pos");
                opMode.multipleTelemetry.update();
            }
        }
        setDrivePowersToZero();
        opMode.robot.travelling = false;
        return true;
    }

    public boolean testDriveToPosition(double targetX, double targetY, double maxPower, double fineMaxPower, double targetHeading, double rotateFactor, double fineThreshold, double targetSlidePos, boolean slideWantMax, boolean collect) {
        collectDone = !collect;
        robotAtLocation = false;

        //opMode.robot.collector.collectDone(collect);
        calculateAndSetDrivePowers(targetX, targetY, maxPower, targetHeading, rotateFactor);
        while (opMode.opModeIsActive() && !testAllMovementDone(fineThreshold < 0 && collect, 0, DRIVE_ERROR_THRESHOLD)) {
            if (driveDone && strafeDone && rotateDone){
                setDrivePowersToZero();

                if(fineThreshold < 0){
                    break;
                }
            } else{
                calculateAndSetDrivePowers(targetX, targetY, maxPower, targetHeading, rotateFactor);
            }
            opMode.multipleTelemetry.addData("Status", "driving to pos");
            opMode.robot.transport.updateTransport(0, targetSlidePos, slideWantMax, collect);
            opMode.robot.collector.getSensorStuff();
            opMode.multipleTelemetry.update();
        }
        if(fineThreshold >= 0){
            collectDone = !collect;
            robotAtLocation = false;

            while(opMode.opModeIsActive() && !testAllMovementDone(collect, 0, fineThreshold)){
                if (driveDone && strafeDone && rotateDone){
                    robotAtLocation = true;
                    setDrivePowersToZero();
                } else{
                    robotAtLocation = false;
                    calculateAndSetDrivePowers(targetX, targetY, fineMaxPower, targetHeading, 0.037);
                }
                opMode.robot.transport.updateTransport(0, targetSlidePos, slideWantMax, collect);
                opMode.multipleTelemetry.addData("Status", "adjusting pos");
                opMode.robot.collector.getSensorStuff();
                opMode.multipleTelemetry.update();
            }
        }
        setDrivePowersToZero();

        return true;
    }

    public void driveToDistance(double targetDistance, double junk){
        distanceError = targetDistance - backDistance.getDistance(DistanceUnit.INCH);
        double drivePower;

            while (opMode.opModeIsActive() && Math.abs(distanceError) > DISTANCE_THRESHOLD) {
                if(Math.abs(opMode.gamepad1.left_stick_x) > 0.1 || Math.abs(opMode.gamepad1.left_stick_y) > 0.1 || Math.abs(opMode.gamepad1.right_stick_y) > 0.1  ) {
                    opMode.robot.scoring = false;
                    break;
                }
                double pastDistance = distanceError;
                distanceError = targetDistance - backDistance.getDistance(DistanceUnit.INCH);
                if(Math.abs(distanceError) > 15 && Math.abs(pastDistance) > 15  && opMode.getClass() == MM_TeleOp.class){
                    opMode.robot.scoring = false;
                    setDrivePowersToZero();
                    break;
                }

                drivePower = distanceError * MM_CONSTANTS.DRIVE_CONSTANTS.DRIVE_P_COEFF;

                flPower = drivePower;
                frPower = drivePower;
                blPower = drivePower;
                brPower = drivePower;

                normalize(.4);
                normalizeForMin(.14);
                setDrivePowers();
            }
            if(Math.abs(distanceError) < DISTANCE_THRESHOLD) {
                opMode.robot.scoring = true;
            }
        setDrivePowersToZero();
        }

    public void calculateAndSetDrivePowers(double targetX, double targetY, double maxPower, double targetHeading, double rotateFactor){
        opMode.robot.navigation.updatePosition();
        xError = targetX - opMode.robot.navigation.getX();
        yError = targetY - opMode.robot.navigation.getY();
        headingError = getHeadingError(targetHeading, opMode.robot.navigation.getHeading());

        double moveAngle = Math.toDegrees(Math.atan2(yError, xError));
        double theta = moveAngle - opMode.robot.navigation.getHeading() + 45;

        double rotate = headingError * rotateFactor;
        double strafe = Math.cos(Math.toRadians(theta)) - Math.sin(Math.toRadians(theta));//xError * DRIVE_P_COEFF;
        double drive = Math.sin(Math.toRadians(theta)) + Math.cos(Math.toRadians(theta));//yError * DRIVE_P_COEFF;

        flPower = drive + strafe - rotate;
        frPower = drive - strafe + rotate;
        blPower = drive - strafe - rotate;
        brPower = drive + strafe + rotate;

        normalize(maxPower);
        //normalizeForMin(.28);

        setDrivePowers();
//        if(MM_OpMode.matchTimer.seconds() >= 27.5){
//            opMode.robot.ascent.prepareFirstLvlAscent();
//            if(MM_OpMode.matchTimer.seconds() >= 29.5){
//                opMode.robot.ascent.ascendFirstLevel();
//            }
//        }

        opMode.multipleTelemetry.addData("zMove angle", moveAngle);
        opMode.multipleTelemetry.addData("zHeading error", headingError);
        opMode.multipleTelemetry.addData("zXError", xError);
        opMode.multipleTelemetry.addData("zYError", yError);
        opMode.multipleTelemetry.addData("zTheta", theta);

    }

    private double getHeadingError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        error = (error >= 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }

    private void setDrivePowersToZero() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    private void setDrivePowers() {
        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    private void normalize(double contextualMaxPower){
        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > contextualMaxPower){
            flPower = (flPower / maxPower) * contextualMaxPower;
            frPower = (frPower / maxPower) * contextualMaxPower;
            blPower = (blPower / maxPower) * contextualMaxPower;
            brPower = (brPower / maxPower) * contextualMaxPower;
        }
    }

    private void normalizeForMin(double minPower) {
        if (Math.abs(flPower) < minPower && Math.abs(frPower) < minPower && Math.abs(blPower) < minPower && Math.abs(brPower) < minPower) {
            double rawMaxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower)));

            double multiplier = minPower / rawMaxPower;
            flPower *= multiplier;
            frPower *= multiplier;
            blPower *= multiplier;
            brPower *= multiplier;
        }
    }

    private boolean allMovementDone(boolean collect, double pivotAngle, double driveThreshold){
        rotateDone = Math.abs(headingError) < HEADING_ERROR_THRESHOLD;
        strafeDone = Math.abs(yError) < driveThreshold;
        driveDone = Math.abs(xError) < driveThreshold;
        if (Math.hypot(yError, xError) <= TANGENT_THRESHOLD){
            driveDone = true;
            strafeDone = true;
        }
        boolean transportDone = opMode.robot.transport.transportMovementDone();
        //boolean transportDone = true; // temp - delete this & uncomment line above

        if (!collectDone && collect) {
            collectDone = opMode.robot.collector.collectDone(pivotAngle);
        }
        //collectDone = true; // temp - delete this & uncomment line above

        opMode.multipleTelemetry.addData("doneRotate", rotateDone);
        opMode.multipleTelemetry.addData("doneStrafe", strafeDone);
        opMode.multipleTelemetry.addData("doneDrive", driveDone);
        opMode.multipleTelemetry.addData("doneTransport", transportDone);

        return (driveDone && strafeDone && rotateDone && (transportDone || (pivotAngle > 80 && driveThreshold == DRIVE_ERROR_THRESHOLD)) && collectDone);
    }

    private boolean testAllMovementDone(boolean collect, double pivotAngle, double driveThreshold){
        rotateDone = Math.abs(headingError) < HEADING_ERROR_THRESHOLD;
        strafeDone = Math.abs(yError) < driveThreshold;
        driveDone = Math.abs(xError) < driveThreshold;
        if (Math.hypot(yError, xError) <= TANGENT_THRESHOLD){
            driveDone = true;
            strafeDone = true;
        }
        boolean transportDone = opMode.robot.transport.transportMovementDone();
        //boolean transportDone = true; // temp - delete this & uncomment line above

        if (!collectDone) {
            collectDone = opMode.robot.collector.testCollectDone(collect);
        }
        //collectDone = true; // temp - delete this & uncomment line above

        opMode.multipleTelemetry.addData("doneRotate", rotateDone);
        opMode.multipleTelemetry.addData("doneStrafe", strafeDone);
        opMode.multipleTelemetry.addData("doneDrive", driveDone);
        opMode.multipleTelemetry.addData("doneTransport", transportDone);

        return (driveDone && strafeDone && rotateDone && (transportDone || (pivotAngle > 80 && driveThreshold == DRIVE_ERROR_THRESHOLD)) && collectDone);
    }

    private void init(){
        if (opMode.getClass() == MM_Autos.class){

        }

        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);

        imu = opMode.hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        backDistance = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
    }
}
