package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autos", group="mm")
public class MM_Autos extends MM_OpMode{

    @Override
    public void runProcedures(){
        if (goal.equals(CHAMBER)){
            chamber();
        } else {
            basket();
        }
    }

    public void chamber(){

        robot.drivetrain.driveToDistance(1);
        //robot.drivetrain.driveInches(-60, 0);
        //TODO add robot dependant chamber code
    }

    public void basket() {
        robot.drivetrain.strafeInches(-14.5, -45);
        //TODO add robot dependant basket code
    }
}
