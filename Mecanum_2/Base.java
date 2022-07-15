package org.firstinspires.ftc.teamcode.Mecanum_2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mecanum_2.Modules.Drive;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Motor;
import org.firstinspires.ftc.teamcode.Utils.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public abstract class Base extends LinearOpMode {
  // Sleep Times
  public ElapsedTime matchTime = new ElapsedTime();

  // Gyro and Angles
  public BNO055IMU gyro;

  public Drive dt = null;

  // Constants and Conversions
  public double targetAngle, currAngle, drive, turn, strafe, multiplier = 1;
  public double initAng = 0;
  public String driveType;

  // Positions and Bounds
  public double dpadTurnSpeed = 0.175, dpadDriveSpeed = 0.2;

  // Button Variables
  public boolean yP = false, yLP = false;
  public boolean aP = false, aLP = false;
  public boolean rP2 = false, rLP2 = false;
  public boolean lP2 = false, lLP2 = false;
  public boolean yP2 = false, yLP2 = false;
  public boolean rSP2 = false, rSLP2 = false;
  public boolean bP2 = false, bLP2 = false;
  public boolean slowDrive = false, fastDrive = false;
  public boolean basicDrive = false;

  public void initHardware(int angle, OpMode m) throws InterruptedException {
    initHardware(0, 0, angle, m);
    initAng = angle;
  }

  public void initHardware(int xPos, int yPos, int angle, OpMode m) throws InterruptedException {
    // Hubs
    List<LynxModule> allHubs;
    allHubs = hardwareMap.getAll(LynxModule.class);

    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    // Motors
    Motor fLeftMotor = new Motor(hardwareMap, "front_left_motor");
    Motor bLeftMotor = new Motor(hardwareMap, "back_left_motor");
    Motor fRightMotor = new Motor(hardwareMap, "front_right_motor");
    Motor bRightMotor = new Motor(hardwareMap, "back_right_motor");

    Motor odoL = new Motor(hardwareMap, "enc_left");
    Motor odoR = new Motor(hardwareMap, "enc_right");
    Motor odoN = new Motor(hardwareMap, "enc_x");

    fLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    fRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    bRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    bLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    // Servo

    // Gyro
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile =
        "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    gyro = hardwareMap.get(BNO055IMU.class, "imu");
    gyro.initialize(parameters);

    // Modules
    dt =
        new Drive(
            fLeftMotor,
            bLeftMotor,
            fRightMotor,
            bRightMotor,
            odoL,
            odoR,
            odoN,
            gyro,
            m,
            xPos,
            yPos,
            angle,
            allHubs);

    initServos();

    // reset constants
    targetAngle = currAngle = drive = turn = strafe = multiplier = 1;
    dpadTurnSpeed = 0.175;
    dpadDriveSpeed = 0.2;
    initAng = angle;
    yP = false;
    yLP = false;
    aP = false;
    aLP = false;
    rP2 = false;
    rLP2 = false;
    lP2 = false;
    lLP2 = false;
    yP2 = false;
    yLP2 = false;
    rSP2 = false;
    rSLP2 = false;
    bP2 = false;
    bLP2 = false;
    slowDrive = false;
    fastDrive = false;
    basicDrive = false;
  }

  public void initHardware(OpMode m) throws InterruptedException {
    initHardware(0, m);
  }

  public void initServos() {}

  // Autonomous Movement (Note that you do not have to insert the current position into any of the
  // weighpoints)
  public void SplinePathConstantHeading(
      ArrayList<Point> pts,
      double heading,
      double driveSpeedCap,
      double xError,
      double yError,
      double angleError,
      int lookAheadDist,
      double timeout) {
    Point curLoc = dt.getCurrentPosition();
    ArrayList<Point> wps = PathGenerator.interpSplinePath(pts, curLoc);
    dt.traversePath(
        wps, heading, driveSpeedCap, false, -1, xError, yError, angleError, lookAheadDist, timeout);
  }

  public void SplinePathConstantHeading(
      ArrayList<Point> pts,
      double heading,
      double driveSpeedCap,
      double powLb,
      double xError,
      double yError,
      double angleError,
      int lookAheadDist,
      double timeout) {
    Point curLoc = dt.getCurrentPosition();
    ArrayList<Point> wps = PathGenerator.interpSplinePath(pts, curLoc);
    dt.traversePath(
        wps,
        heading,
        driveSpeedCap,
        true,
        powLb,
        xError,
        yError,
        angleError,
        lookAheadDist,
        timeout);
  }

  public void LinearPathConstantHeading(
      ArrayList<Point> pts,
      double heading,
      double driveSpeedCap,
      double powLb,
      double xError,
      double yError,
      double angleError,
      int lookAheadDist,
      double timeout) {
    Point curLoc = dt.getCurrentPosition();
    ArrayList<Point> wps = new ArrayList<>();
    wps.add(curLoc);
    wps.addAll(pts);
    wps = PathGenerator.generateLinearSpline(wps);
    dt.traversePath(
        wps, heading, driveSpeedCap, powLb, xError, yError, angleError, lookAheadDist, timeout);
  }

  public void turnTo(double targetAngle, long timeout, double powerCap, double minDifference) {
    dt.turnTo(targetAngle, timeout, powerCap, minDifference);
  }

  public void turnTo(double targetAngle, long timeout, double powerCap) {
    dt.turnTo(targetAngle, timeout, powerCap, 2);
  }

  public void turnTo(double targetAngle, long timeout) {
    turnTo(targetAngle, timeout, 0.7);
  }

  public void moveToPosition(
      double targetXPos,
      double targetYPos,
      double targetAngle,
      double posAccuracy,
      double angleAccuracy,
      double timeout) {
    dt.moveToPosition(
        targetXPos, targetYPos, targetAngle, posAccuracy, posAccuracy, angleAccuracy, timeout);
  }

  public void moveToPosition(
      double targetXPos, double targetYPos, double targetAngle, double timeout) {
    moveToPosition(targetXPos, targetYPos, targetAngle, 2, 2, timeout);
  }

  public void moveToPosition(
      double targetXPos,
      double targetYPos,
      double targetAngle,
      double posAccuracy,
      double timeout) {
    moveToPosition(targetXPos, targetYPos, targetAngle, posAccuracy, 2, timeout);
  }

  // Function implementing Points
  public void moveToPosition(
      Point p, double xAccuracy, double yAccuracy, double angleAccuracy, double timeout) {
    dt.moveToPosition(p.xP, p.yP, p.ang, xAccuracy, yAccuracy, angleAccuracy, timeout);
  }

  public void moveToPosition(Point p, double posAccuracy, double angleAccuracy, double timeout) {
    dt.moveToPosition(p.xP, p.yP, p.ang, posAccuracy, posAccuracy, angleAccuracy, timeout);
  }

  public void moveToPosition(Point p, double timeout) {
    moveToPosition(p.xP, p.yP, p.ang, 2, 2, timeout);
  }

  public void moveToPosition(Point p, double posAccuracy, double timeout) {
    moveToPosition(p.xP, p.yP, p.ang, posAccuracy, 2, timeout);
  }

  // Driver Controlled Movemement
  public void computeDrivePowers(Gamepad gamepad) {
    if (basicDrive) {
      driveType = "Robot Centric";

      if (gamepad.dpad_right) {
        dt.driveRobotCentric(0, dpadTurnSpeed, 0);
      } else if (gamepad.dpad_left) {
        dt.driveRobotCentric(0, -dpadTurnSpeed, 0);
      } else if (gamepad.dpad_up) {
        dt.driveFieldCentric(
            targetAngle + currAngle,
            0,
            0.01 * Angle.angleDifference(currAngle, targetAngle + 90),
            0);
      } else {
        dt.driveRobotCentric(drive, turn, strafe);
      }
    } else {
      driveType = "Field Centric";

      if (gamepad.dpad_right) {
        dt.driveFieldCentric(targetAngle + currAngle, 0, dpadTurnSpeed, 0);
      } else if (gamepad.dpad_left) {
        dt.driveFieldCentric(targetAngle + currAngle, 0, -dpadTurnSpeed, 0);
      } else if (gamepad.dpad_up) {
        dt.driveFieldCentric(targetAngle + currAngle, -dpadDriveSpeed, 0, 0);
      } else if (gamepad.dpad_down) {
        dt.driveFieldCentric(targetAngle + currAngle, dpadDriveSpeed, 0, 0);
      } else {
        dt.driveFieldCentric(targetAngle + currAngle, drive, turn, strafe);
      }
    }
  }

  // Misc Utility Functions
  public String formatDegrees(double degrees) {
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }

  public double floor(double rawInput) {
    if (slowDrive) {
      return ((int) (rawInput * 5.5)) / 11.0;
    } else if (fastDrive) {
      return rawInput;
    }
    return ((int) (rawInput * 9)) / 11.0;
  }

  public double turnFloor(double rawInput) {
    return ((int) (rawInput * 15)) / 20.0;
  }

  public String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  // Other Functions
  public double normalizeThreeDigits(double d) {
    return (int) (d * 1000) / 1000.;
  }

  @Override
  public abstract void runOpMode() throws InterruptedException;
}
