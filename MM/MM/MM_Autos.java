package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.DRIVE_CONSTANTS.BASE_ROTATE_FACTOR;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_Collector.haveSample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autos", group = "mm")
public class MM_Autos extends MM_OpMode {
    public ElapsedTime aprilTagTime = new ElapsedTime();


    @Override
    public void runProcedures() {
        if (goal.equals(CHAMBER)) {
            chamber();
        } else {
            //robot.navigation.setPosition(-36, -60, 60);
            basket();
        }
    }

    public void chamber() {
        robot.drivetrain.driveToPosition(0, -36.1, .4, 0, alliance == 1? -90: -90 + 180, BASE_ROTATE_FACTOR, -1, 92, 16.1, false, false);
        robot.drivetrain.driveToDistance(4.7, .6);
        robot.scoreSpecimen();

        //start pushing
        robot.drivetrain.driveToPosition(6, -39,.8, 0,alliance == 1? -90: 90, BASE_ROTATE_FACTOR, -1, 92, 0, false, false);
        robot.drivetrain.driveToPosition(32, -39, .8, 0, alliance == 1? 180:0, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);
        robot.drivetrain.driveToPosition(34.5, -12, .8, 0, alliance == 1? 180:0, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);
        robot.drivetrain.driveToPosition(40.5, -12, .8, 0, alliance == 1? 180:0, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);
        robot.drivetrain.driveToPosition(40.5, -56, .8, 0, alliance == 1? 180:0, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);

        //        robot.drivetrain.driveToPosition(37.5, -24, .8, 0, alliance == 1? 90: 90 + 180, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);
//        robot.drivetrain.driveToPosition(37.5, -57, .8, 0, alliance == 1? 90: 90 + 180, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);
//        robot.drivetrain.driveToPosition(37.5, -24, .8, 0, alliance == 1? 90: 90 + 180, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);
        robot.drivetrain.driveToPosition(45, -12, .8, 0, alliance == 1? 180:0, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);
        robot.drivetrain.driveToPosition(50, -12, .8, 0, alliance == 1? 180:0, BASE_ROTATE_FACTOR, -1, 92, .2, false, false);

        robot.drivetrain.driveToPosition(55, -56,.8, 0,alliance == 1? 180: 0, BASE_ROTATE_FACTOR, -1, 92, 0, false, false);
        //robot.drivetrain.driveToPosition(46.76, -57, .8, 0, alliance == 1? 90: 90 + 180, BASE_ROTATE_FACTOR, -1, 92,0, false, false);
        robot.drivetrain.driveToDistance(4.7, .8);
        robot.collectSpec();
        robot.drivetrain.driveToPosition(-4, -41.1, .8, 0, alliance == 1? -90: -90 + 180, BASE_ROTATE_FACTOR, -1, 92, 16.1, false, false);
        robot.drivetrain.driveToDistance(4.7, .6);
        robot.scoreSpecimen();
        //end pushing
        robot.drivetrain.driveToPosition(0, -39,.8, 0,alliance == 1? -90: -90 + 180, BASE_ROTATE_FACTOR, -1, 92, 0, false, false);
        robot.drivetrain.driveToPosition(55, -56,.8, 0,alliance == 1? 180: 0, BASE_ROTATE_FACTOR, -1, 92, 0, false, false);
        //robot.drivetrain.driveToPosition(46.76, -57, .8, 0, alliance == 1? 90: 90 + 180, BASE_ROTATE_FACTOR, -1, 92,0, false, false);
        robot.drivetrain.driveToDistance(4.7, .8);
        robot.collectSpec();
        robot.drivetrain.driveToPosition(4, -41.1, .8, 0, alliance == 1? -90: -90 + 180, BASE_ROTATE_FACTOR, -1, 92, 16.1, false, false);
        robot.drivetrain.driveToDistance(4.7, .6);
        robot.scoreSpecimen();

    }

    public void basket() {

        driveToBasketAndScore(.036);

        lookAtAprilTag();

        multipleTelemetry.addData("Status", "Trying to Collect");
        multipleTelemetry.update();
        robot.drivetrain.driveToPosition(-46.6 * alliance, -38.5 * alliance, .5, .2, alliance == 1 ? 90 : 90 + 180, BASE_ROTATE_FACTOR, .32, -14.2, 5.5, false, true);
        if(!haveSample){
            robot.collector.uncollect();
        }

        //multipleTelemetry.addData("Status", "re-align collect");

        //robot.drivetrain.driveToPosition(-46.6, -39.5, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);
        driveToBasketAndScore(0.036);

        lookAtAprilTag();

        multipleTelemetry.addData("Status", "Trying to Collect");
        multipleTelemetry.update();
        robot.drivetrain.driveToPosition(-56.6 * alliance, -38.5 * alliance, .5, .3, alliance == 1 ? 90 : 90 + 180, BASE_ROTATE_FACTOR,.32,  -14.2, 5.5, false, true);
        if(!haveSample){
            robot.collector.uncollect();
        }

        driveToBasketAndScore(.043);

        lookAtAprilTag();

        robot.drivetrain.driveToPosition(-58.75 * alliance, -46.8 * alliance, .5, .3, alliance==1?110.86: 110.86 +180, .05, .37, -13, 14, false, true);
        if(!haveSample){
            robot.collector.uncollect();
        }
        driveToBasketAndScore( .07);


        robot.drivetrain.driveToPosition(-33 * alliance, -10 * alliance, .8, .3, alliance==1?0: 180, .036, .37, -24, .5, false, false);
        robot.ascent.ascendFirstLevel();
        while (opModeIsActive()){}
    }

    private void lookAtAprilTag() {
        multipleTelemetry.addData("Status", "Driving to April Tag");
        robot.drivetrain.driveToPosition(-48 * alliance, -48 * alliance, .8, .3, alliance == 1 ? 90 : 90 + 180, .043, -1, 0, 5.5, false, false);

        aprilTagTime.reset();
        while (opModeIsActive() && aprilTagTime.milliseconds() <= 350) {
            multipleTelemetry.addData("Status", "looking, at apriltag");
            robot.navigation.updatePosition(true);
            multipleTelemetry.update();
        }
    }

    public void driveToBasketAndScore(double rotateFactor) {
        if (robot.collector.haveSample()) {
            multipleTelemetry.addData("Status", "Trying to Score");
            multipleTelemetry.update();
            robot.drivetrain.driveToPosition(-53.7 * alliance, -55 * alliance, .8, .3, alliance == 1 ? 32 : 32 + 180, rotateFactor, .37, 93, 52, false, false);
            if(robot.collector.haveSample()) {
                robot.collector.score();
            }
        }
    }

}
