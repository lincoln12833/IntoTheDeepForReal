package org.firstinspires.ftc.teamcode.MM;

import static org.firstinspires.ftc.teamcode.MM.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM.MM_OpMode.previousGamepad1;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MM_Drivetrain {

    private final MM_OpMode opMode;

    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;

    private IMU imu;

    private GoBildaPinpointDriver odometryController;

    private Rev2mDistanceSensor backDistance;
    Pose2D odometryPos;

    public static double MAX_POWER = 1;
    public static double SLOW_POWER = .5;

    public static double MAX_TURN_POWER = .5;
    public static double MIN_TURN_POWER = .15;
    public static double GYRO_TURN_P_COEFF = .016;
    public static double HEADING_ERROR_THRESHOLD = 3;

    private final double DRIVE_ERROR_THRESHOLD = 1;
    private final double DRIVE_P_COEFF = 0.03125;

    private final double DISTANCE_THRESHOLD = .5;

    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;

    private double drivePower;
    private double strafePower;
    private double rotatePower;

    private double heading;

    private boolean slow = false;
    private boolean strafeDone = false;
    private boolean driveDone = false;
    private boolean rotateDone = false;

    double targetPos;
    double targetDrivePos;
    double targetStrafePos;
    double headingError = 100;
    double driveInchesError;
    double distance;
    double distanceError;
    double strafeInchesError;


    MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveWithSticks() {
        drivePower = -opMode.gamepad1.left_stick_y;
        strafePower = opMode.gamepad1.left_stick_x;
        rotatePower = opMode.gamepad1.right_stick_x;

        flPower = drivePower + strafePower + rotatePower;
        frPower = drivePower - strafePower - rotatePower;
        blPower = drivePower - strafePower + rotatePower;
        brPower = drivePower + strafePower - rotatePower;

        if (currentGamepad1.a && !previousGamepad1.a) {
            slow = !slow;
        }

        normalize(MAX_POWER);

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

    public void driveInches(double targetInches, int targetHeading) {
        odometryController.update();
        odometryPos = odometryController.getPosition();

        targetPos = targetInches + odometryPos.getX(DistanceUnit.INCH);
        headingError = 100;
        while (opMode.opModeIsActive() && (!allMovementDone(false))) {
           updateDrivePowers(targetInches, targetHeading);
        }
        setDrivePowers(0);
    }

    public void driveInches(double targetInches, int targetHeading, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect) {
        odometryController.update();
        odometryPos = odometryController.getPosition();

        targetDrivePos = targetInches + odometryPos.getX(DistanceUnit.INCH);


        headingError = 100;
        while (opMode.opModeIsActive() && (!allMovementDone(collect))) {
            updateDrivePowers(targetInches, targetHeading);
            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
            opMode.robot.collector.handleCollect(collect);
        }
        setDrivePowers(0);

    }

    public void updateDrivePowers(double targetInches, int targetHeading){

        odometryController.update();
        odometryPos = odometryController.getPosition();

        headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
        driveInchesError = targetPos - odometryPos.getX(DistanceUnit.INCH);


        rotatePower = rotateDone? 0: MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
        drivePower = driveDone? 0: MAX_POWER * driveInchesError * DRIVE_P_COEFF;

        flPower = drivePower - rotatePower;
        frPower = drivePower + rotatePower;
        blPower = drivePower - rotatePower;
        brPower = drivePower + rotatePower;

        normalize(MAX_TURN_POWER);
        normalizeForMin(.28);
        opMode.telemetry.addLine("normalized");

        setDrivePowers();

        opMode.telemetry.addData("heading error", headingError);
        opMode.telemetry.addData("inches error", driveInchesError);
    }

    //hi

    public void driveToDistance(double targetDistance, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect) {
        while (Math.abs(distanceError) > DISTANCE_THRESHOLD) {
            updateDistancePowers(targetDistance);
            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
            opMode.robot.collector.handleCollect(collect);
        }
        setDrivePowers(0);
    }

    public void driveToDistance(double targetDistance) {
        while (Math.abs(distanceError) > DISTANCE_THRESHOLD) {
            updateDistancePowers(targetDistance);
        }
        setDrivePowers(0);
    }


    public void updateDistancePowers(double targetDistance) {
        distance = backDistance.getDistance(DistanceUnit.INCH);
        distanceError = targetDistance - distance;

        drivePower = distanceError * MAX_POWER * DRIVE_P_COEFF;

        flPower = drivePower;
        frPower = drivePower;
        blPower = drivePower;
        brPower = drivePower;

        normalizeForMin(.14);

        setDrivePowers();
    }

    public void strafeInches(double targetInches, int targetHeading, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect){
        odometryController.update();
        odometryPos = odometryController.getPosition();

        targetPos = targetInches + odometryPos.getY(DistanceUnit.INCH);
        headingError = 100;
        if (slideTargetInches > 0) {
            opMode.robot.transport.slide.setPower(.5);
        }
        if (targetPivotAngle > -25) {
            opMode.robot.transport.pivot.pivot.setPower(1);
        }
        while ( opMode.opModeIsActive() && !allMovementDone(collect)){
            updateStrafePowers(targetInches, targetHeading);
            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
            opMode.robot.collector.handleCollect(collect);
        }
        setDrivePowers(0);
    }

    public void strafeInches(double targetInches, int targetHeading){
        odometryController.update();
        odometryPos = odometryController.getPosition();
        while (opMode.opModeIsActive() && !allMovementDone(false)){
            updateStrafePowers(targetInches, targetHeading);
        }
        setDrivePowers(0);
    }

    private void updateStrafePowers(double targetInches, int targetHeading) {
        odometryController.update();
        odometryPos = odometryController.getPosition();

        headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
        strafeInchesError = targetPos - (odometryPos.getY(DistanceUnit.INCH));


        rotatePower = rotateDone? 0: MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
        strafePower = strafeDone? 0: MAX_POWER * strafeInchesError * DRIVE_P_COEFF;

        flPower = strafePower - rotatePower;
        frPower = -strafePower + rotatePower;
        blPower = -strafePower - rotatePower;
        brPower = strafePower + rotatePower;

        normalize(MAX_TURN_POWER);
        normalizeForMin(.28);

        opMode.telemetry.addData("strafe error", strafeInchesError);
        opMode.telemetry.addData("strafe power", strafePower);
        opMode.telemetry.addData("Heading error", headingError);
        opMode.telemetry.addData("rotate power", rotatePower);
        opMode.telemetry.addData("flPower", flPower);
        opMode.telemetry.addData("frPower", frPower);
        opMode.telemetry.addData("blPower", blPower);
        opMode.telemetry.addData("bbrPower", brPower);

        setDrivePowers();
    }

    public void rotateToAngle(int targetAngle) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        headingError = getHeadingError(targetAngle, odometryPos.getHeading(AngleUnit.DEGREES));


        while (opMode.opModeIsActive() && Math.abs(headingError) > HEADING_ERROR_THRESHOLD) {
            double power = headingError * GYRO_TURN_P_COEFF * MAX_TURN_POWER;

            flPower = -(power);
            frPower = power;
            blPower = -(power);
            brPower = power;

//            normalizeForMin(MIN_TURN_POWER);
//            normalize(MAX_TURN_POWER);

            setDrivePowers();

            odometryController.update();
            odometryPos = odometryController.getPosition();

            headingError = getHeadingError(targetAngle, odometryPos.getHeading(AngleUnit.DEGREES));

            opMode.telemetry.addData("power", power);
            opMode.telemetry.addData("error", headingError);
//            opMode.telemetry.addData("heading", heading);
            opMode.telemetry.update();
        }
        setDrivePowers(0);
    }

    public void doEverything(double targetDriveInches, double targetStrafeInches, int targetHeading, double targetPivotAngle, double slideTargetInches, boolean slideWantMax, boolean collect){

        targetDrivePos = targetDriveInches + odometryPos.getX(DistanceUnit.INCH);
        targetStrafePos = targetStrafeInches + odometryPos.getY(DistanceUnit.INCH);

        headingError = 100;
        while (opMode.opModeIsActive() && !allMovementDone(collect)) {
            updateDriveAndStrafe(targetHeading);
            opMode.robot.transport.updateTransport(targetPivotAngle, slideTargetInches, slideWantMax);
            opMode.robot.collector.handleCollect(collect);
        }
    }

    public void updateDriveAndStrafe(int targetHeading){
        odometryController.update();
        odometryPos = odometryController.getPosition();

        driveInchesError = targetDrivePos - odometryPos.getX(DistanceUnit.INCH);
        headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
        strafeInchesError = targetStrafePos - (odometryPos.getY(DistanceUnit.INCH));


        rotatePower = rotateDone? 0: MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
        strafePower = strafeDone? 0: MAX_POWER * strafeInchesError * DRIVE_P_COEFF;
        drivePower = driveDone? 0: MAX_POWER * driveInchesError * DRIVE_P_COEFF;


        flPower = drivePower + strafePower - rotatePower;
        frPower = drivePower - strafePower + rotatePower;
        blPower = drivePower - strafePower - rotatePower;
        brPower = drivePower + strafePower + rotatePower;

        normalize(MAX_TURN_POWER);
        normalizeForMin(.28);

        opMode.telemetry.addData("strafe error", strafeInchesError);
        opMode.telemetry.addData("strafe power", strafePower);
        opMode.telemetry.addData("Heading error", headingError);
        opMode.telemetry.addData("rotate power", rotatePower);
        opMode.telemetry.addData("flPower", flPower);
        opMode.telemetry.addData("frPower", frPower);
        opMode.telemetry.addData("blPower", blPower);
        opMode.telemetry.addData("bbrPower", brPower);

        setDrivePowers();
    }

    private double getHeadingError(int targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        error = (error > 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }

    private void setDriveMode(DcMotor.RunMode runToPosition) {
        flMotor.setMode(runToPosition);
        frMotor.setMode(runToPosition);
        blMotor.setMode(runToPosition);
        brMotor.setMode(runToPosition);
    }

    private void setDrivePowers(double power) {
        flMotor.setPower(power);
        frMotor.setPower(power);
        blMotor.setPower(power);
        brMotor.setPower(power);
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
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }
    }

    private void normalizeForMin(double minPower) {
        if (flPower < minPower && frPower < minPower && blPower < minPower && brPower < minPower) {
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
        strafeDone = Math.abs(strafeInchesError) < DRIVE_ERROR_THRESHOLD;
        driveDone = Math.abs(driveInchesError) < DRIVE_ERROR_THRESHOLD;
        boolean transportDone = opMode.robot.transport.transportMovementDone();

        opMode.telemetry.addData("rotate Done", rotateDone);
        opMode.telemetry.addData("strafe done", strafeDone);
        opMode.telemetry.addData("drive done", driveDone);
        opMode.telemetry.addData("transport done", transportDone);
        opMode.telemetry.update();

        return (driveDone && strafeDone && rotateDone && transportDone && opMode.robot.collector.collectDone(collect));
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

        odometryController = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometryController.setOffsets(1.905, 2.54);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //TODO Check to be sure of directions

        odometryController.resetPosAndIMU();


        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometryController.update();
        odometryPos = odometryController.getPosition();

        backDistance = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
    }
}
