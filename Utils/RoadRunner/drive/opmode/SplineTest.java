package org.firstinspires.ftc.teamcode.Utils.RoadRunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.analysis.function.Tan;
import org.firstinspires.ftc.teamcode.Utils.RoadRunner.drive.Mecanum;
import org.firstinspires.ftc.teamcode.Utils.RoadRunner.drive.Tank;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Tank drive = new Tank(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-11, 30), Math.toRadians(90))
                .build();

        drive.followTrajectory(traj);

        sleep(500);

        drive.turn(Math.toRadians(180));

        sleep(500);


        Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(-9, -28), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj2);

        sleep(500);

        drive.setMotorPowers(0, 0, 0, 0);
    }
}
