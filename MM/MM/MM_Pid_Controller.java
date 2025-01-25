package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.PID_CONSTANTS.D_CO_EFF;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.PID_CONSTANTS.I_CO_EFF;
import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.PID_CONSTANTS.P_CO_EFF;

public class MM_Pid_Controller {
    private final MM_OpMode opMode;

    private double driveIntegral;
    private double strafeIntegral;
    private double driveDerivative;
    private double strafeDerivative;
    private double lastDrivePower;
    private double lastStrafepower;
    double[] powers = new double[2];

    public MM_Pid_Controller(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public double[] calculatePowers(double driveIn, double strafeIn){
        driveIntegral += driveIn;
        driveDerivative = driveIn - lastDrivePower;
        strafeDerivative = strafeIn - lastStrafepower;

        lastDrivePower = driveIn;
        lastStrafepower = strafeIn;

        powers[0] = (driveIn * P_CO_EFF) + (driveIntegral * I_CO_EFF) + (driveDerivative * D_CO_EFF);
        powers[1] = (strafeIn * P_CO_EFF) + (strafeIntegral * I_CO_EFF) + (driveDerivative * D_CO_EFF);

        return powers;
    }
}
