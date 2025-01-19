package org.firstinspires.ftc.teamcode.MM.MM;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MM_Navigation {
    public static final double TAG_FLIP_THRESHOLD = .2;
    private final MM_OpMode opMode;

    public MM_VisionPortal visionPortal;
    public GoBildaPinpointDriver odometryController;

    Pose2D currentPos;
    Pose2D AprilTagPos;
    double pastExtrinsicY;

    MM_Navigation(MM_OpMode opMode){
        this.opMode = opMode;
        visionPortal = new MM_VisionPortal(opMode);

        odometryController = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometryController.setOffsets(49.08, 6.135);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odometryController.resetPosAndIMU();

        odometryController.update();
        currentPos = odometryController.getPosition();

    }

    public void updatePosition(){
        if (AprilTagPos != null) {
            pastExtrinsicY = AprilTagPos.getY(DistanceUnit.INCH);
        }
        AprilTagPos = visionPortal.setPosFromApriltag();
        currentPos = odometryController.getUpdatedPositon();

        if(AprilTagPos != null ){
            currentPos = odometryController.getUpdatedPositon();

            opMode.multipleTelemetry.addData("ODOrobot x", getX());
            opMode.multipleTelemetry.addData("ODOrobot y", getY());
            opMode.multipleTelemetry.addData("ODOrobot yaw", getHeading());

            opMode.multipleTelemetry.addData("APRILrobot x", AprilTagPos.getX(DistanceUnit.INCH));
            opMode.multipleTelemetry.addData("APRILrobot y", AprilTagPos.getY(DistanceUnit.INCH));
            opMode.multipleTelemetry.addData("APRILrobot yaw", AprilTagPos.getHeading(AngleUnit.DEGREES));
            opMode.multipleTelemetry.addData("APRILrobot x", AprilTagPos.getX(DistanceUnit.INCH));
            if (Math.abs((Math.abs(MM_VisionPortal.intrinsicX - MM_VisionPortal.previousIntrinsicX)) - (Math.abs(AprilTagPos.getY(DistanceUnit.INCH) - pastExtrinsicY) )) <= TAG_FLIP_THRESHOLD || opMode.opModeInInit() || opMode.getClass() == MM_TeleOp.class) { //Math.abs(AprilTagPos.getHeading(AngleUnit.DEGREES) - currentPos.getHeading(AngleUnit.DEGREES)) <= Math.abs(currentPos.getHeading(AngleUnit.DEGREES) * .25)
                odometryController.setPosition(AprilTagPos);
                opMode.multipleTelemetry.addData("set From Apriltag", true);
                currentPos = odometryController.getUpdatedPositon();
            } else {
                opMode.multipleTelemetry.addData("set From Apriltag", false);
            }
        }

        currentPos = odometryController.getUpdatedPositon();

        opMode.multipleTelemetry.addData("robot x", getX());
        opMode.multipleTelemetry.addData("robot y", getY());
        opMode.multipleTelemetry.addData("robot yaw", getHeading());
    }

    public double getX() {
        return currentPos.getX(DistanceUnit.INCH);
    }

    public double getY() {
        return currentPos.getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return currentPos.getHeading(AngleUnit.DEGREES);
    }

    public void setPosition(double xPos, double yPos, double yawPos){
        odometryController.setPosition(new Pose2D(DistanceUnit.INCH, xPos, yPos, AngleUnit.DEGREES, yawPos));
    }

}
