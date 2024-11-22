package org.firstinspires.ftc.teamcode.MM;

public class MM_Transport {
    private final MM_OpMode opMode;

    public static MM_Slide slide;
    public static MM_Pivot pivot;

    public static double pivotAngle = 0;

    MM_Transport(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public void chamberPos(){
        
    }

    public static void updateTransport(double pivotAngle, double slideInches, boolean wantMax){
        pivot.updatePivot(pivotAngle);
        slide.updateSlide(wantMax, slideInches);
    }

    public void runTransport(){
        pivot.controlPivot();
        slide.runSlide();

    }

    public void home(){
        slide.home();
        pivot.home();
    }
    public void init() {
        pivot = new MM_Pivot(opMode);
        slide = new MM_Slide(opMode);
    }

}
