package org.firstinspires.ftc.teamcode.Mecanum_2.Modules;

import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Point;

public class Odometry {
  // Constants
  public final double WHEEL_DIAMETER = 4;
  public final double INTER_WHEEL_WIDTH = 16;
  public final double INTER_WHEEL_LENGTH = 14;
  public final double TICKS_PER_DRIVE_ROTATION = 1120;
  public final double TICKS_PER_ENCODER_ROTATION = 1120;
  public final double ENCODER_WHEEL_DIAMETER = 2;
  private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
  private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 2.0 * (ENCODER_WHEEL_DIAMETER * 0.5);
  private final double ENCODER_WIDTH = 12.0;

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

  // https://github.com/Beta8397/virtual_robot/blob/master/TeamCode/src/org/firstinspires/ftc/teamcode/EncBot.java
  public void updatePosition(double l, double r, double n, double ang) {
    double dR = r - lastRightEnc;
    double dL = l - lastLeftEnc;
    double dN = n - lastNormalEnc;

    double rightDist = dR * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
    double leftDist = -dL * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
    double dyR = 0.5 * (rightDist + leftDist);
    double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;
    double dxR = -dN * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
    double avgHeadingRadians = Math.toRadians(angle) + headingChangeRadians / 2.0;
    double cos = Math.cos(avgHeadingRadians);
    double sin = Math.sin(avgHeadingRadians);

    xPos += dxR * sin + dyR * cos;
    yPos += -dxR * cos + dyR * sin;
    angle = Angle.normalize(ang);
    lastNormalEnc = n;
    lastLeftEnc = l;
    lastRightEnc = r;
  }

  public double normalizeAngle(double rawAngle) {
    double scaledAngle = rawAngle % 360;
    if (scaledAngle < 0) {
      scaledAngle += 360;
    }

    if (scaledAngle > 180) {
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

  public void resetOdometry() {
    resetOdometry(0, 0, 0);
  }

  public void resetOdometry(Point p) {
    resetOdometry(p.xP, p.yP, p.ang);
  }

  public void resetOdometry(double xPos, double yPos, double angle) {
    this.xPos = xPos;
    this.yPos = yPos;
    this.angle = angle;
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }

  private String format(double num) {
    return String.format("%.3f", num);
  }
}
