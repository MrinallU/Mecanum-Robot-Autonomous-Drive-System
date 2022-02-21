package org.firstinspires.ftc.teamcode.State_Championship_2022.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.State_Championship_2022.Modules.Camera;
import org.firstinspires.ftc.teamcode.State_Championship_2022.Base;

@Autonomous(name="Secondary_Blue_Autonomous", group="Autonomous")
public class Secondary_Blue_Autonomous extends Base {
    int pos = 0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        Camera camera = new Camera(hardwareMap);
        sleep(2000);

        pos = camera.readBarcode("blueSecondary");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

//        if(pos == 0){
//            arm.moveBottom();
//        }else if(pos == 1){
//            arm.moveMid();
//        }else{
//            arm.moveTop();
//        }

        // aligns robot for inital deposit
        Trajectory traj = driveTrain.trajectoryBuilder(new Pose2d(), true) // sweeper side means front
                .splineTo(new Vector2d(-11, 30), Math.toRadians(90))
                .build();

        driveTrain.followTrajectory(traj);

        sleep(500);

        // todo make roadrunner compatible turnTo
        driveTrain.turn(Math.toRadians(180)); // this turn is relative

        sleep(500);

        // park in the warehouse
        Trajectory traj2 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), true) // make the start of the trajectory the curr robot pose always!
                .splineTo(new Vector2d(-9, -28), Math.toRadians(0))
                .build();

        driveTrain.followTrajectory(traj2);

        sleep(500);

        driveTrain.setMotorPowers(0, 0, 0, 0);
    }
}
