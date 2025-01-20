package org.firstinspires.ftc.teamcode.MM.MM;

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
        //    robot.drivetrain.driveInches(24, 0);

//        robot.drivetrain.driveInches(4, 0);
//        robot.drivetrain.rotateToAngle(179);
//        robot.drivetrain.driveToDistance(2.2, 90, 20, false,  false);
//        robot.transport.slide.setTargetInches(36);
//        robot.drivetrain.driveInches(12, 179);
//        robot.transport.home();
//        robot.drivetrain.driveInches(25, 179);
//        robot.drivetrain.strafeInches(38, 179);
        //TODO TEST
    }

    public void basket() {

            driveToBasketAndScore();

            lookAtAprilTag();

            multipleTelemetry.addData("Status", "Trying to Collect");
            robot.drivetrain.driveToPosition(-46.6 * alliance, -39.5 * alliance, .2, alliance == 1?91.5:91.5 + 180, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);
            //multipleTelemetry.addData("Status", "re-align collect");

            //robot.drivetrain.driveToPosition(-46.6, -39.5, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);
            driveToBasketAndScore();

            lookAtAprilTag();

            multipleTelemetry.addData("Status", "Trying to Collect");
            robot.drivetrain.driveToPosition(-56.6 * alliance, -39.5 * alliance, .3, alliance == 1?91.5:91.5 + 180, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);

            driveToBasketAndScore();
//            //second collect
//            multipleTelemetry.addData("Status", "Trying to Collect");
//            robot.drivetrain.driveToPosition(-56.6, -39.5, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);
//            driveToRedBasketAndScore();
//
//            //third collect
//            //robot.drivetrain.driveToPosition(-56, -40.5, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 6, false, true);
//            //driveToRedBasketAndScore();
//            robot.drivetrain.driveToPosition(-50, -50, 32, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 0, false, true);

        //pre-penfield code (before inch change)

//        //robot.drivetrain.doEverything(5, -8, -45, 90, robot.transport.slide.MAX_TICKS, true, false);
//        robot.drivetrain.strafeInches(-8, -45, 90, robot.transport.slide.MAX_TICKS, true, false); //TODO test to see if doEverything() works then delete these lines
//        robot.drivetrain.driveInches(-5, -45, 90, robot.transport.slide.MAX_TICKS, true, false);
////        sleep(30000); hihihihihihi
//        robot.collector.score();
//        robot.drivetrain.driveInches(6, -45, 90, 14, false, false);
//        robot.drivetrain.driveInches(7, 0, -13, 7.5, false, true);
//        if(robot.collector.haveSample()) {
//            robot.drivetrain.driveInches(-13, -45, 90, robot.transport.slide.MAX_TICKS, true, false);
//            robot.collector.score();
//            robot.drivetrain.driveInches(13, 0, -5, 5, false, false);
//        } else {
//            //robot.drivetrain.driveInches(-2, 0, -5, 8.5, false, false);
//            robot.drivetrain.strafeInches(-2, 0);
//        }
//        robot.drivetrain.strafeInches(-7.5, 0, -5, 8.5, false, false);
//        robot.drivetrain.strafeInches(-.1, 0, -12, 8.5, false, true);
//        robot.drivetrain.strafeInches(6.5, 0, 90, robot.transport.slide.MAX_TICKS, true, false);
//        robot.drivetrain.driveInches(-11.5, -50);
//        robot.collector.score();
//        robot.drivetrain.driveInches(13, 29, 90, 11, false, false);
//        robot.drivetrain.driveInches(.2, 29, -12, 8, false, true);
////        robot.drivetrain.rotateToAngle(-90);
//        robot.drivetrain.strafeInches(46, -90);
//        robot.drivetrain.driveToDistance(1, 90, 8, false, false);
        //TODO TEST
    }

    private void lookAtAprilTag() {
        multipleTelemetry.addData("Status", "Driving to April Tag");
        robot.drivetrain.driveToPosition(-47 * alliance, -47 * alliance, .5, alliance==1?90:90+180, .043, 0, 5.5, false, false);

        aprilTagTime.reset();
        while(opModeIsActive() && aprilTagTime.milliseconds() <= 500) {
            multipleTelemetry.addData("Status", "looking, at apriltag");
            robot.navigation.updatePosition(true);
            multipleTelemetry.update();
        }
    }

    public void driveToBasketAndScore() {
        if (robot.collector.haveSample()) {
            multipleTelemetry.addData("Status", "Trying to Score");
            robot.drivetrain.driveToPosition(-53.7 * alliance, -55 * alliance, .5, alliance == 1?32: 32+180, MM_TestTrigNav.ROTATE_FACTOR, 93, 52, false, false);
            robot.collector.score();
        }
    }

    public void driveToBlueBasketAndScore() {
        if (robot.collector.haveSample()) {
            multipleTelemetry.addData("Status", "Trying to Score");
            robot.drivetrain.driveToPosition(53.7, 55, .5, -148, MM_TestTrigNav.ROTATE_FACTOR, 93, 52, false, false);
            robot.collector.score();
        }
    }
}
