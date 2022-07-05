package org.firstinspires.ftc.teamcode.Mecanum_2.Modules;

import org.firstinspires.ftc.teamcode.Utils.Point;

public class Odometry {
    // Constants
    private static final double TICKS_PER_ROTATION = 360 * 4; // CPR is 360 but 4 revolutions
    private static final double WHEEL_CIRCUMFERENCE = 58 * Math.PI; // In mm
    private static final double DISTANCE_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / 25.4; // In inches
    public static final double DISTANCE_PER_TICK = (DISTANCE_PER_REVOLUTION / TICKS_PER_ROTATION);
    private static double DISTANCE_BETWEEN_WHEELS = 15.06; // In in
    private static final double NORMAL_ENCODER_DRIFT = 0.001275;

    private boolean verbose = false;
    public String outStr = "";

    // Variables
    private double xPos, yPos, angle;
    private double lastLeftEnc, lastRightEnc, lastNormalEnc;

    public Odometry(double xPos, double yPos, double angle) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.angle = angle;
    }

    public Odometry(double angle) {
        this.xPos = 0;
        this.yPos = 0;
        this.angle = angle;
    }

    public Odometry() {
        this(0);
    }

    public void updatePosition(double r, double l, double n, double ang) {
        double c = 11 * 2 * Math.PI;
        double dL = l  - lastLeftEnc;
        double dR = r - lastRightEnc;
        double dTheta = ((dR - dL) / DISTANCE_BETWEEN_WHEELS);
        double dM = n - lastNormalEnc;
        // double dM = n - lastNormalEnc - (c / (2 * Math.PI) * dTheta); // becuase the middle wheel is
        // not free spinning you dont have to subtract c.

        double dS = (dR + dL) / 2.0;
        // double avgTheta = Math.toRadians(angle) + dTheta / 2.0;
        double avgTheta = Math.toRadians(ang); // true angle from IMU
        double dY = dS * Math.sin(avgTheta) - dM *
                Math.cos(avgTheta);
        double dX = dS * Math.cos(avgTheta) + dM *
                Math.sin(avgTheta);

        xPos += dX;
        yPos += dY;
        angle = ang;
        // angle += Math.toDegrees(dTheta);

        lastLeftEnc = l;
        lastRightEnc = r;
        lastNormalEnc = n;

        // Set string so values can be passed to telemetry
        if(verbose){
            outStr = "dLeft: " + format(dL) + "\ndRight: " + format(dR) + "\ndNormal: " + format(dM);
        }
        else{
            outStr = "xPos: " + format(xPos) + "\nyPos: " + format(yPos) + "\nAngle: " + format(angle);
        }
    }

    public double normalizeAngle(double rawAngle) {
        double scaledAngle = rawAngle % 360;
        if ( scaledAngle < 0 ) {
            scaledAngle += 360;
        }

        if ( scaledAngle > 180 ) {
            scaledAngle -= 360;
        }

        return scaledAngle;
    }

    public String displayPositions() {
        return outStr;
    }

    public double getAngle() {
        return angle;
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public void resetOdometry(){
        resetOdometry(0, 0, 0);
    }

    public void resetOdometry(Point p){
        resetOdometry(p.xP, p.yP, p.ang);
    }

    public void resetOdometry(double xPos, double yPos, double angle){
        this.xPos = xPos;
        this.yPos = yPos;
        this.angle = angle;
    }

    public void addToDistBetweenWheels(double addDist) {
        DISTANCE_BETWEEN_WHEELS += addDist;
    }

    public void setAngle(double angle){
        this.angle = angle;
    }

    private String format(double num){
        return String.format("%.3f", num);
    }

}
