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
        robot.drivetrain.driveInches(-40, 179, 90, 0, false, false);


        //robot.drivetrain.driveInches(-60, 0);
        //robot.transport.chamberPos();
        //score specimen


        //robot.drivetrain.strafeInches(39, 0, 5.47736, 36.667424, false, false);


        //TODO add robot dependant chamber code
    }

    public void basket() {
        robot.drivetrain.strafeInches(-14.5, -45);
        //TODO add robot dependant basket code
    }
}
