package org.firstinspires.ftc.teamcode.State_Championship_2022.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.State_Championship_2022.Modules.Camera;
import org.firstinspires.ftc.teamcode.State_Championship_2022.Base;

@Autonomous(name="Secondary_Blue_Autonomous", group="State")
public class Secondary_Blue_Autonomous extends Base {
    int pos = 0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0, true);
        initServosAuto();
        Camera camera = new Camera(hardwareMap);
        sleep(2000);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        pos = camera.readBarcode("blueSecondary");

        if (isStopRequested()) return;
        arm.moveToPosition(300);
        // todo finish inital deposit
        // aligns robot for inital deposit
        Trajectory traj = driveTrain.trajectoryBuilder(new Pose2d(), true)
                .back(13)
                .build();

        driveTrain.followTrajectory(traj);
        sleep(250);

        driveTrain.turn(Math.toRadians(90));
        sleep(250);




        double dist = 0, tickOffset = 0;
        if (pos == 0) {
            arm.moveBottomBlueSecondary();
            dist = 14;
            tickOffset = 0;
        } else if (pos == 1) {
            arm.moveMidBlueSecond();
            dist = 16;
            tickOffset = 120;
        } else {
            arm.moveTop();
            dist = 18;
            tickOffset = 160;
        }


        sleep(1000);

        Trajectory deposit = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), false)
                .forward(dist)
                .build();
        driveTrain.followTrajectory(deposit);

        sleep(250);
        arm.dump();
        sleep(250);

        driveTrain.moveTicks((1700+tickOffset), 4000, 0.5, 20, this, true);
        container.dumpBlock();
        arm.moveToPosition(300);

        sleep(500);



        driveTrain.turn(Math.toRadians(180));
        sleep(250);
        driveTrain.setPoseEstimate(new Pose2d(0, 0, driveTrain.getPoseEstimate().getHeading()));

        arm.sweepPos();
        sleep(1500);

        sweeper.sweep();
        Trajectory sweepSpline = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), false)
                .forward(12)
                .build();
        driveTrain.followTrajectory(sweepSpline);

        sleep(1000);
        arm.container.sweepBlock();
        sweeper.stop();

        arm.moveToPosition(300);

        Trajectory alignBack = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), false)
                .back(10)
                .build();
        driveTrain.followTrajectory(alignBack);

        sleep(250);
        driveTrain.turnToV2(25, 2000, 1);

        sleep(250);
        Trajectory align2 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), false)
                .back(6)
                .build();
        driveTrain.followTrajectory(align2);

        driveTrain.turnToV2(120, 2000, 1);


        sleep(100);
        driveTrain.moveTicks(1680, 4000, 0.5, 20, this, false);
        sleep(250);
        driveTrain.turnToV2(65, 2000, 1);
        sleep(100);
        arm.moveTopSecondCycleBlue();
        driveTrain.moveTicks(650, 4000, 0.5, 20, this, false);
        sleep(700);
        arm.dump();
        sleep(500);
        arm.container.dumpBlock();
        arm.moveToPosition(300);
        driveTrain.moveTicks(2200, 4000, 0.8, 20, this, true);
        sleep(1000);











        // move back






        // park



        // park in the warehouse
//        Trajectory traj4 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), false) // make the start of the trajectory the curr robot pose always!
//                .splineTo(new Vector2d(-10, 15), Math.toRadians(0))
//                .build();
//
//        driveTrain.followTrajectory(traj4);
//
//        sleep(250);

        // align with wall and relocalize with distance sensors

        // turnTo(90)
        // driveTrain.setPoseEstimate(new Pose2d( getDistanceSensorValues));

        // splineTo the freight stack


        // drive through if freight not found


        // realign to barrier


        // cross the barrier


        // turn and relocalize
        //turnTo(90)
        // driveTrain.setPoseEstimate(new Pose2d( getDistanceSensorValues));


        // arm up and navigate to hub


        // dump


        // park

        driveTrain.setMotorPowers(0, 0, 0, 0);
    }
}
