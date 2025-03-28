package org.firstinspires.ftc.teamcode.MM.MM;

public class MM_Transport {
    private final MM_OpMode opMode;

    public MM_Slide slide;
    public MM_Pivot pivot;

    public static double pivotAngle = 0;
    public static int maxSlideTicksForAngle = 0;

    MM_Transport(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public boolean transportMovementDone(){
        opMode.multipleTelemetry.addData("donePivot", pivot.pivotMovementDone());
        opMode.multipleTelemetry.addData("doneSlide", slide.slideMovementDone());
        return slide.slideMovementDone() && pivot.pivotMovementDone();
    }

    public void updateTransport(double pivotAngle, double slideInches, boolean wantMax, boolean collect){
        if (MM_Drivetrain.robotAtLocation || !collect) {
            pivot.updatePivot(pivotAngle);
        } else{
            pivot.setAngle(pivotAngle + 20);
        }
        slide.updateSlide(wantMax, slideInches);
    }

    public void runTransport(){
        pivot.controlPivot();
        slide.runSlide();

    }



    public void home(){
        slide.home();
        pivot.home();
        while(!transportMovementDone()){}
    }
    public void init() {
        pivot = new MM_Pivot(opMode);
        slide = new MM_Slide(opMode);
    }

}
