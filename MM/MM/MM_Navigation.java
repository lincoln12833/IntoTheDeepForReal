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

        odometryController.setOffsets(53.975, 3.175);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odometryController.resetPosAndIMU();

        odometryController.update();
        currentPos = odometryController.getPosition();
    }

    public void updatePosition(){
        currentPos = odometryController.getUpdatedPositon();
        opMode.multipleTelemetry.addData("xOdom", round2Dec(getX()));
        opMode.multipleTelemetry.addData("yOdom", round2Dec(getY()));
        opMode.multipleTelemetry.addData("yawOdom", round2Dec(getHeading()));

        if (AprilTagPos != null) {
            pastExtrinsicY = AprilTagPos.getY(DistanceUnit.INCH);
        }
        AprilTagPos = visionPortal.setPosFromApriltag();
        if(AprilTagPos != null ){
            opMode.multipleTelemetry.addData("xApril", round2Dec(AprilTagPos.getX(DistanceUnit.INCH)));
            opMode.multipleTelemetry.addData("yApril", round2Dec(AprilTagPos.getY(DistanceUnit.INCH)));
            opMode.multipleTelemetry.addData("yawApril", round2Dec(AprilTagPos.getHeading(AngleUnit.DEGREES)));
            if (MM_OpMode.currentGamepad1.b && !MM_OpMode.previousGamepad1.b) {
                odometryController.setPosition(AprilTagPos);
            }

            if (Math.abs(intrinsicDiff() - extrinsicDiff()) <= TAG_FLIP_THRESHOLD || opMode.opModeInInit() || opMode.getClass() == MM_TeleOp.class) {
//                if (opMode.opModeInInit()) {
//                    odometryController.setPosition(AprilTagPos);
//                }
                opMode.multipleTelemetry.addData("set From Apriltag", true);
                //currentPos = odometryController.getUpdatedPositon();
            } else {
                opMode.multipleTelemetry.addData("set From Apriltag", false);
            }
        }
        else { //just here for dashboard
            opMode.multipleTelemetry.addData("xApril", "");
            opMode.multipleTelemetry.addData("yApril", "");
            opMode.multipleTelemetry.addData("yawApril", "");
        }
        currentPos = odometryController.getUpdatedPositon();

//        opMode.multipleTelemetry.addData("xRobot", round2Dec(getX()));
//        opMode.multipleTelemetry.addData("yRobot", round2Dec(getY()));
//        opMode.multipleTelemetry.addData("yawRobot", round2Dec(getHeading()));
    }

    private double extrinsicDiff() {
        return Math.abs(AprilTagPos.getY(DistanceUnit.INCH) - pastExtrinsicY);
    }

    private static double intrinsicDiff() {
        return Math.abs(MM_VisionPortal.intrinsicX - MM_VisionPortal.previousIntrinsicX);
    }

    private double round2Dec(double inDouble) {
        return Math.round(inDouble * 100) / 100.0;
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
