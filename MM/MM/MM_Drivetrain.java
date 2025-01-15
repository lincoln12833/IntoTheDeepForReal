package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_OpMode.previousGamepad1;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double HEADING_ERROR_THRESHOLD = 3;

    private final double DRIVE_ERROR_THRESHOLD = 1;
    private final double DRIVE_P_COEFF = 0.015625; //prev 0.03125

    private final double TANGENT_THRESHOLD = 0.5;

    private final double DISTANCE_THRESHOLD = .5;

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
        double drivePower = -opMode.gamepad1.left_stick_y;
        double strafePower = opMode.gamepad1.left_stick_x;
        double rotatePower = -opMode.gamepad1.right_stick_x; //left is a positive rotation

        flPower = drivePower + strafePower - rotatePower;
        frPower = drivePower - strafePower + rotatePower;
        blPower = drivePower - strafePower - rotatePower;
        brPower = drivePower + strafePower + rotatePower;

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

    public boolean driveToPosition(double targetX, double targetY, double targetHeading, double rotateFactor, double pivotAngle, double targetSlidePos, boolean slideWantMax, boolean collect) {
        opMode.robot.collector.collectDone(collect);
        calculateAndSetDrivePowers(targetX, targetY, targetHeading, rotateFactor);
        while (opMode.opModeIsActive() && !allMovementDone(collect)) {
            if (driveDone && strafeDone && rotateDone){
                setDrivePowersToZero();
            } else{
                calculateAndSetDrivePowers(targetX, targetY, targetHeading, rotateFactor);
            }

            opMode.robot.transport.updateTransport(pivotAngle, targetSlidePos, slideWantMax);
            opMode.multipleTelemetry.update();
        }

        return true;
    }

    public void calculateAndSetDrivePowers(double targetX, double targetY, double targetHeading, double rotateFactor){
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

        normalize(MAX_POWER);
        //normalizeForMin(.28);

        setDrivePowers();

        opMode.multipleTelemetry.addData("move angle", moveAngle);
        opMode.multipleTelemetry.addData("heading error", headingError);
        opMode.multipleTelemetry.addData("xError", xError);
        opMode.multipleTelemetry.addData("yError", yError);
        opMode.multipleTelemetry.addData("theta", theta);
    }

//    public void driveInches(double targetInches, int targetHeading) {
//        MM_Navigation.odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        targetPos = targetInches + odometryPos.getX(DistanceUnit.INCH);
//        headingError = 100;
//        while (opMode.opModeIsActive() && (!allMovementDone(false))) {
//           updateDrivePowers(targetInches, targetHeading);
//        }
//        setDrivePowers(0);
//    }
//
//    public void driveInches(double targetInches, int targetHeading, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect) {
//        odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        targetPos = targetInches + odometryPos.getX(DistanceUnit.INCH);
//
//        if (slideTargetInches > 0) {
//            opMode.robot.transport.slide.setPower(.5);
//        }
//        if (targetPivotAngle > -25) {
//            opMode.robot.transport.pivot.pivot.setPower(1);
//        }
//
//        headingError = 100;
//        while (opMode.opModeIsActive() && (!allMovementDone(collect))) {
//            updateDrivePowers(targetInches, targetHeading);
//            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
//        }
//        setDrivePowers(0);
//
//    }
//
//    public void updateDrivePowers(double targetInches, int targetHeading){
//
//        odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
//        driveInchesError = targetPos - odometryPos.getX(DistanceUnit.INCH);
//
//
//        rotatePower = rotateDone? 0: MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
//        drivePower = driveDone? 0: MAX_POWER * driveInchesError * DRIVE_P_COEFF;
//
//        flPower = drivePower - rotatePower;
//        frPower = drivePower + rotatePower;
//        blPower = drivePower - rotatePower;
//        brPower = drivePower + rotatePower;
//
//        normalize(1);
//        normalizeForMin(.28);
//        opMode.multipleTelemetry.addLine("normalized");
//
//        setDrivePowers();
//
//        opMode.multipleTelemetry.addData("heading error", headingError);
//        opMode.multipleTelemetry.addData("inches error", driveInchesError);
//    }
//
//    public void driveToPosition(double targetFieldPosX, double targetFieldPosY, double targetHeading) {
//        opMode.robot.updatePosition();
//        xError = MM_Robot.position.getX(DistanceUnit.INCH) - targetFieldPosX;
//        yError = MM_Robot.position.getY(DistanceUnit.INCH) - targetFieldPosY;
//        headingError = MM_Robot.position.getHeading(AngleUnit.DEGREES) - targetHeading;
//
//        drivePower = yError * DRIVE_P_COEFF * MAX_POWER;
//        strafePower = xError * DRIVE_P_COEFF * MAX_POWER;
//        //rotatePower
//    }
//
//    public void driveToDistance(double targetDistance, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect) {
//        while (Math.abs(distanceError) > DISTANCE_THRESHOLD) {
//            updateDistancePowers(targetDistance);
//            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
//            opMode.robot.collector.handleCollect(collect);
//        }
//        setDrivePowers(0);
//    }
//
//    public void driveToDistance(double targetDistance) {
//        while (Math.abs(distanceError) > DISTANCE_THRESHOLD) {
//            updateDistancePowers(targetDistance);
//        }
//        setDrivePowers(0);
//    }
//
//
//    public void updateDistancePowers(double targetDistance) {
//        distance = backDistance.getDistance(DistanceUnit.INCH);
//        distanceError = targetDistance - distance;
//
//        drivePower = distanceError * MAX_POWER * DRIVE_P_COEFF;
//
//        flPower = drivePower;
//        frPower = drivePower;
//        blPower = drivePower;
//        brPower = drivePower;
//
//        normalizeForMin(.14);
//
//        setDrivePowers();
//    }
//
//    public void strafeInches(double targetInches, int targetHeading, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect){
//        odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        targetPos = targetInches + odometryPos.getY(DistanceUnit.INCH);
//        headingError = 100;
//        if (slideTargetInches > 0) {
//            opMode.robot.transport.slide.setPower(.5);
//        }
//        if (targetPivotAngle > -25) {
//            opMode.robot.transport.pivot.pivot.setPower(1);
//        }
//        while ( opMode.opModeIsActive() && !allMovementDone(collect)){
//            updateStrafePowers(targetInches, targetHeading);
//            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
//        }
//        setDrivePowers(0);
//    }
//
//    public void strafeInches(double targetInches, int targetHeading){
//        odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        targetPos = targetInches + odometryPos.getY(DistanceUnit.INCH);
//        headingError = 100;
//
//        while (opMode.opModeIsActive() && !allMovementDone(false)){
//            updateStrafePowers(targetInches, targetHeading);
//        }
//        setDrivePowers(0);
//    }
//
//    private void updateStrafePowers(double targetInches, int targetHeading) {
//        odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
//        strafeInchesError = targetPos - (odometryPos.getY(DistanceUnit.INCH));
//
//
//        rotatePower = rotateDone? 0: MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
//        strafePower = strafeDone? 0: MAX_POWER * strafeInchesError * DRIVE_P_COEFF;
//
//        flPower = strafePower - rotatePower;
//        frPower = -strafePower + rotatePower;
//        blPower = -strafePower - rotatePower;
//        brPower = strafePower + rotatePower;
//
//        normalize(MAX_TURN_POWER);
//        normalizeForMin(.28);
//
//        opMode.multipleTelemetry.addData("strafe error", strafeInchesError);
//        opMode.multipleTelemetry.addData("strafe power", strafePower);
//        opMode.multipleTelemetry.addData("Heading error", headingError);
//        opMode.multipleTelemetry.addData("rotate power", rotatePower);
//        opMode.multipleTelemetry.addData("flPower", flPower);
//        opMode.multipleTelemetry.addData("frPower", frPower);
//        opMode.multipleTelemetry.addData("blPower", blPower);
//        opMode.multipleTelemetry.addData("brPower", brPower);
//
//        setDrivePowers();
//    }
//
//    public void rotateToAngle(int targetAngle) {
//        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        headingError = getHeadingError(targetAngle, odometryPos.getHeading(AngleUnit.DEGREES));
//
//
//        while (opMode.opModeIsActive() && Math.abs(headingError) > HEADING_ERROR_THRESHOLD) {
//            double power = headingError * GYRO_TURN_P_COEFF * MAX_TURN_POWER;
//
//            flPower = -(power);
//            frPower = power;
//            blPower = -(power);
//            brPower = power;
//
////            normalizeForMin(MIN_TURN_POWER);
////            normalize(MAX_TURN_POWER);
//
//            setDrivePowers();
//
//            odometryController.update();
//            odometryPos = odometryController.getPosition();
//
//            headingError = getHeadingError(targetAngle, odometryPos.getHeading(AngleUnit.DEGREES));
//
//            opMode.multipleTelemetry.addData("power", power);
//            opMode.multipleTelemetry.addData("error", headingError);
////            opMode.multipleTelemetry.addData("heading", heading);
//            opMode.multipleTelemetry.update();
//        }
//        setDrivePowers(0);
//    }
//
//    public void doEverything(double targetDriveInches, double targetStrafeInches, int targetHeading, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect){
//        odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        targetDrivePos = targetDriveInches + odometryPos.getX(DistanceUnit.INCH);
//        targetStrafePos = targetStrafeInches + odometryPos.getY(DistanceUnit.INCH);
//
//        if (slideTargetInches > 0) {
//            opMode.robot.transport.slide.setPower(.5);
//        }
//        if (targetPivotAngle > -25) {
//            opMode.robot.transport.pivot.pivot.setPower(1);
//        }
//
//        headingError = 100;
//        while (opMode.opModeIsActive() && !allMovementDone(collect)) {
//            updateDriveAndStrafe(targetHeading);
//            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
//            opMode.robot.collector.handleCollect(collect);
//        }
//    }
//
//    public void updateDriveAndStrafe(int targetHeading){
//        odometryController.update();
//        odometryPos = odometryController.getPosition();
//
//        driveInchesError = targetDrivePos - odometryPos.getX(DistanceUnit.INCH);
//        headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
//        strafeInchesError = targetStrafePos - (odometryPos.getY(DistanceUnit.INCH));
//
//
//        rotatePower = 0; //rotateDone? 0: MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
//        strafePower = strafeDone? 0: MAX_POWER * strafeInchesError * DRIVE_P_COEFF;
//        drivePower = driveDone? 0: MAX_POWER * driveInchesError * DRIVE_P_COEFF;
//
//
//        flPower = drivePower + strafePower - rotatePower;
//        frPower = drivePower - strafePower + rotatePower;
//        blPower = drivePower - strafePower - rotatePower;
//        brPower = drivePower + strafePower + rotatePower;
//
//        normalize(MAX_TURN_POWER);
//        normalizeForMin(.28);
//
//        opMode.multipleTelemetry.addData("strafe error", strafeInchesError);
//        opMode.multipleTelemetry.addData("strafe power", strafePower);
//        opMode.multipleTelemetry.addData("Heading error", headingError);
//        opMode.multipleTelemetry.addData("rotate power", rotatePower);
//        opMode.multipleTelemetry.addData("flPower", flPower);
//        opMode.multipleTelemetry.addData("frPower", frPower);
//        opMode.multipleTelemetry.addData("blPower", blPower);
//        opMode.multipleTelemetry.addData("brPower", brPower);
//
//        setDrivePowers();
//    }
//
    private double getHeadingError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        error = (error > 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
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

    private boolean allMovementDone(boolean collect){
        rotateDone = Math.abs(headingError) < HEADING_ERROR_THRESHOLD;
        strafeDone = Math.abs(yError) < DRIVE_ERROR_THRESHOLD;
        driveDone = Math.abs(xError) < DRIVE_ERROR_THRESHOLD;
        if (Math.hypot(yError, xError) <= TANGENT_THRESHOLD){
            driveDone = true;
            strafeDone = true;
        }
        boolean transportDone = opMode.robot.transport.transportMovementDone();

        collectDone = opMode.robot.collector.collectDone(collect);

        opMode.multipleTelemetry.addData("rotate Done", rotateDone);
        opMode.multipleTelemetry.addData("strafe done", strafeDone);
        opMode.multipleTelemetry.addData("drive done", driveDone);
        opMode.multipleTelemetry.addData("transport done", transportDone);

        return (driveDone && strafeDone && rotateDone && transportDone && collectDone);
    }

    private void init(){
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
