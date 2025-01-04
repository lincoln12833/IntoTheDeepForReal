package org.firstinspires.ftc.teamcode.MM.MM;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MM_Navigation {
    private final MM_OpMode opMode;

    public MM_VisionPortal visionPortal;
    public GoBildaPinpointDriver odometryController;
    Pose2D currentPos;

    MM_Navigation(MM_OpMode opMode){
        this.opMode = opMode;
        visionPortal = new MM_VisionPortal(opMode);

        odometryController = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometryController.setOffsets(1.905, 2.54);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odometryController.resetPosAndIMU();

        odometryController.update();
        currentPos = odometryController.getPosition();
    }

    public void updatePosition(){
        Pose2D AprilTagPos = visionPortal.setPosFromApriltag();

        if(AprilTagPos != null){
            odometryController.setPosition(AprilTagPos);
        }

        odometryController.update();
        currentPos = odometryController.getPosition();

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

}
