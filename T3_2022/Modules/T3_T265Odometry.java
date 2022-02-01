
package org.firstinspires.ftc.teamcode.T3_2022.Modules;
import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import 	android.util.Log;

public class T3_T265Odometry {
    // Motors
    public String outStr = "";

    // Variables
    private double xPos, yPos, angle;
    private HardwareMap hardwareMap;
    private String tag = "T265_Status";

    Pose2d startPos;
    private T265Camera slamera;
    public T265Camera.PoseConfidence poseConfidence;


    public T3_T265Odometry(double xPos, double yPos, double angle, HardwareMap hardwareMap) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.angle = angle;
        this.hardwareMap = hardwareMap;
        startPos =  new Pose2d(xPos, yPos, Angle.degrees_to_radians(angle));
    }

    public T3_T265Odometry(double angle, HardwareMap hardwareMap) {
        this.xPos = 0;
        this.yPos = 0;
        this.angle = angle;
        this.hardwareMap = hardwareMap;
        startPos =  new Pose2d(0, 0, Angle.degrees_to_radians(angle));
    }

    public T3_T265Odometry(HardwareMap hardwareMap) {
        this(0, hardwareMap);
    }


    public void updatePosition() {
        // Get update
        T265Camera.CameraUpdate rawPose = slamera.getLastReceivedCameraUpdate();
        Rotation2d rot = rawPose.pose.getRotation();
        Translation2d translation = new Translation2d(rawPose.pose.getTranslation().getX() / 0.0254,
                rawPose.pose.getTranslation().getY() / 0.0254);

        // Apply the needed transformations
        xPos = translation.getX(); yPos = translation.getY(); angle = rot.getRadians();
        Pose2d currPose = transformBy(startPos); // transform
        xPos = currPose.getX(); yPos = currPose.getY(); angle = currPose.getHeading();
        poseConfidence = rawPose.confidence;

        outStr = "xPos: " + format(xPos) + "\nyPos: " + format(yPos) + "\nAngle: " + format((Angle.normalize(Angle.radians_to_degrees(angle))));
    }

    public void initializeT265(){
        Log.d(tag, "Initializing T265");
        if (slamera == null) {
            Log.d(tag, "Slamera was null.");
            slamera = new T265Camera(
                    new Transform2d(
                            new Translation2d(startPos.getX(), startPos.getY()),
                            new Rotation2d(startPos.getHeading())
                    ),
                    0,
                    hardwareMap.appContext
            );
        }
            while (!slamera.isStarted()) {
                Log.w(tag, "Camera is not ready, starting...");
                sleep(1000); // replace if needed.
                slamera.start();
            }
    }

    public void setPose(double x, double y, double angle){
        Pose2d newPose = calculateTransformation(new Pose2d(x, y, angle));
        xPos = newPose.getX(); yPos = newPose.getY(); angle = newPose.getHeading();
    }


    public Pose2d transformBy(Pose2d b) {
        double cosT = Math.cos(angle);
        double sinT = Math.sin(angle);
        return new Pose2d(
                xPos + b.getX() * cosT - b.getY() * sinT,
                yPos + b.getX() * sinT + b.getY() * cosT,
                angle + b.getHeading()
        );
    }

    public Pose2d calculateTransformation(Pose2d r) {
        double cosT = Math.cos(angle);
        double sinT = Math.sin(angle);
        return new Pose2d(
                cosT * (r.getX() - xPos) + sinT * (r.getY() - yPos),
                sinT * (xPos - r.getX()) + cosT * (r.getY() - yPos),
                r.getHeading() - angle
        );
    }

    public void stopT265(){
        slamera.stop();
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


    public void setAngle(double angle){
        this.angle = angle;
    }

    private String format(double num){
        return String.format("%.3f", num);
    }
}
