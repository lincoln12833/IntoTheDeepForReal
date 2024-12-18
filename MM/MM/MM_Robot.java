package org.firstinspires.ftc.teamcode.MM.MM;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MM_Robot{
    private final MM_OpMode opMode;
    
    public MM_Drivetrain drivetrain;
    public MM_Transport transport;
    public MM_Collector collector;
    public MM_VisionPortal visionPortal;

    public static double robotX;
    public static double robotY; //TODO add robot heading
    public static Pose2D position;



    MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }



    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        transport = new MM_Transport(opMode);
        collector = new MM_Collector(opMode);
        visionPortal = new MM_VisionPortal(opMode);
    }


    public void updatePosition(){
        if(visionPortal.setPosFromApriltag()){
            drivetrain.updatePinpoint();
        }

        drivetrain.setPositionFromPinpoint();

        opMode.multipleTelemetry.addData("robot x", position.getX(DistanceUnit.INCH));
        opMode.multipleTelemetry.addData("robot y", position.getY(DistanceUnit.INCH));
        opMode.multipleTelemetry.addData("robot yaw", position.getHeading(AngleUnit.DEGREES));

    }

}