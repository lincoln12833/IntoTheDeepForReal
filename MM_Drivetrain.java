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

    public static double MAX_POWER = 1;
    public static double SLOW_POWER = .5;

    public static double MAX_TURN_POWER = .5;
    public static double MIN_TURN_POWER = .15;
    public static double GYRO_TURN_P_COEFF = .016;
    public static double HEADING_ERROR_THRESHOLD = 10;

    private final double DRIVE_ERROR_THRESHOLD = .5;
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

    Pose2D odometryPos;

    MM_Drivetrain(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void driveWithSticks(){
        drivePower = -opMode.gamepad1.left_stick_y;
        strafePower = opMode.gamepad1.left_stick_x;
        rotatePower = opMode.gamepad1.right_stick_x;

        flPower = drivePower + strafePower + rotatePower;
        frPower = drivePower - strafePower - rotatePower;
        blPower = drivePower - strafePower + rotatePower;
        brPower = drivePower + strafePower - rotatePower;

        if (currentGamepad1.a && !previousGamepad1.a){
            slow = !slow;
        }

        normalize(MAX_POWER);

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    public void driveInches(double targetInches, int targetHeading){
        odometryController.update();
        odometryPos = odometryController.getPosition();

        double targetPos = targetInches + odometryPos.getX(DistanceUnit.INCH);

        double headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
        double inchesError = targetPos - Math.abs(odometryPos.getX(DistanceUnit.INCH));

        while (opMode.opModeIsActive() && ( Math.abs(inchesError) > DRIVE_ERROR_THRESHOLD || Math.abs(headingError) > HEADING_ERROR_THRESHOLD)){
            odometryController.update();
            odometryPos = odometryController.getPosition();

            headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
            inchesError = targetInches - odometryPos.getX(DistanceUnit.INCH);
            opMode.telemetry.addData("heading error", headingError);
            opMode.telemetry.addData("inches error", inchesError);
            opMode.telemetry.update();

            rotatePower = MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
            drivePower = MAX_POWER * inchesError * DRIVE_P_COEFF;

            flPower = drivePower - rotatePower;
            frPower = drivePower + rotatePower;
            blPower = drivePower - rotatePower;
            brPower = drivePower + rotatePower;

            normalize(MAX_TURN_POWER);
            
            setDrivePowers();

        }

    }

    public void driveToDistance(double targetDistance){
        double distance = backDistance.getDistance(DistanceUnit.INCH);
        double distanceError = targetDistance - distance;

        while(opMode.opModeIsActive() && Math.abs(distanceError) > DISTANCE_THRESHOLD){
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
        setDrivePowers(0);
    }

    public void strafeInches(double targetInches, int targetHeading){
        odometryController.update();
        odometryPos = odometryController.getPosition();

        double targetPos = targetInches + odometryPos.getY(DistanceUnit.INCH);

        double headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
        double inchesError = targetPos - odometryPos.getY(DistanceUnit.INCH);

        while (opMode.opModeIsActive() && (Math.abs(inchesError) >  DRIVE_ERROR_THRESHOLD || Math.abs(headingError) > HEADING_ERROR_THRESHOLD)){
            odometryController.update();
            odometryPos = odometryController.getPosition();

            headingError = getHeadingError(targetHeading, odometryPos.getHeading(AngleUnit.DEGREES));
            inchesError = targetInches - odometryPos.getY(DistanceUnit.INCH);

            rotatePower = MAX_TURN_POWER * headingError * GYRO_TURN_P_COEFF;
            strafePower = MAX_POWER * inchesError * DRIVE_P_COEFF;

            flPower = strafePower - rotatePower;
            frPower = -strafePower + rotatePower;
            blPower = -strafePower - rotatePower;
            brPower = strafePower + rotatePower;

            normalize(MAX_TURN_POWER);

            normalizeForMin(.14);

            setDrivePowers();
        }

    }

    public void rotateToAngle(int targetAngle) {
        setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = getHeadingError(targetAngle, heading);

        while (opMode.opModeIsActive() && Math.abs(error) > HEADING_ERROR_THRESHOLD) {
            double power = error * GYRO_TURN_P_COEFF * MAX_TURN_POWER;

            flPower = -(power);
            frPower = power;
            blPower = -(power);
            brPower = power;

            normalizeForMin(MIN_TURN_POWER);
            normalize(MAX_TURN_POWER);

            setDrivePowers();

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = getHeadingError(targetAngle, heading);

            opMode.telemetry.addData("power", power);
            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("heading", heading);
            opMode.telemetry.update();
        }
        setDrivePowers(0);
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

        slow = (!previousGamepad1.a && currentGamepad1.a)? !slow: slow; //DO NOT CHANGE ANDROID WAS WRONG (I triple checked this)

        if (maxPower > contextualMaxPower){
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        if (slow){
            flPower *= SLOW_POWER;
            frPower *= SLOW_POWER;
            blPower *= SLOW_POWER;
            brPower *= SLOW_POWER;
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

    private void init(){
        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

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
        odometryPos = odometryController.getPosition();

        backDistance = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
    }
}
