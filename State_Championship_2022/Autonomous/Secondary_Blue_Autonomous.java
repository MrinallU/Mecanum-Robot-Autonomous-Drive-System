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

        pos = camera.readBarcode("blueSecondary");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;
        arm.moveToPosition(300);
        // todo finish inital deposit
        // aligns robot for inital deposit
        Trajectory traj = driveTrain.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-20, 25), Math.toRadians(90))
                .build();

        driveTrain.followTrajectory(traj);

        sleep(250);

        driveTrain.turn(Math.toRadians(180));
        sleep(250);

        // lower arm
        if(pos == 0){
            arm.moveBottom();
        }else if(pos == 1){
            arm.moveMid();
        }else{
            arm.moveTop();
        }
        sleep(1000);

        // drive in to dump
        Trajectory traj2 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), false)
                .forward(12)
                .build();
        driveTrain.followTrajectory(traj2);

        sleep(250);

        // dump and reblock
        arm.dump();
        sleep(750);
        arm.container.dumpBlock();


        // move back
        Trajectory traj3 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), false)
                .back(50)
                .build();
        driveTrain.followTrajectory(traj3);


        // park

        sleep(250);

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
