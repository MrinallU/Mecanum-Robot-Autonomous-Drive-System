package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;

@Autonomous(name="T3_Secondary_Red_Autonomous", group="Autonomous")
public class T3_Secondary_Red_Autonomous extends T3_Base {
    int pos =  1;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();
        sleep(3000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        //pos = camera.readBarcode("redSecondary");


        if(pos == 0){
            arm.moveBottomRedSecond();
            telemetry.addData("Wobble Level: ", "Bottom");
            telemetry.addData("Shipping Element Placement: ", "☒ ☐ ☐");
            elementDiagram = "☒ ☐ ☐";
            telemetry.update();
        }else if(pos == 1){
            arm.moveMidRedSecond();
            telemetry.addData("Wobble Level: ", "Middle");
            telemetry.addData("Shipping Element Placement: ", "☐ ☒ ☐");
            elementDiagram = "☐ ☒ ☐";
            telemetry.update();
        }else if(pos == 2){
            arm.moveTop();
            telemetry.addData("Wobble Level: ", "Top");
            telemetry.addData("Shipping Element Placement: ", "☐ ☐ ☒");
            elementDiagram = "☐ ☐ ☒";
            telemetry.update();
        }

        moveTicksBack(300, 4000, 0.5, 20,this);
        sleep(250);


        turnToV2(86, 4000, this);
        sleep(250);


        if(pos == 0){
            moveTicksBack(730, 4000, 0.4, 20, this); //bottom deposit
        }else if(pos == 1){
            moveTicksBack(800, 4000, 0.4, 20, this); //mid and top deposit
        }else{
            moveTicksBack(900, 4000, 0.4, 20, this); //mid and top deposit
        }

        sleep(250);
        arm.dump();
        sleep(500);

        // move into barrier
        yTo(40, 4000, 0.7, 3, this, true, true); // @Parth tune this value
        container.dumpBlock();
        arm.sweepPos(); // reset arm
        sleep(250);
        turnToV2(60, 2000, this); // turn to freight stack
        sweeper.sweep();

        moveTicksFront(500, 4000, 0.4, 20, this); // sweep freight
//        moveTicksFront(440, 4000, 0.4, 20, this); // sweep freight
        sleep(2000);
        sweeper.stop();
        container.sweepBlock();
        sweeper.dump();
        sleep(2000);
        sweeper.stop();

        moveTicksBack(500, 4000, 0.4, 20, this); // sweep freight
//
//        arm.moveTop();

        turnToV2(86, 4000, this);
//        sleep(250);
//
//        yTo(-20, 4000, 0.7, 3, this, true, true); // @Parth tune this value



        while(opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            telemetry.addLine("cam pos " + odometry.outStr);
            telemetry.addLine("imu angle " + getAngle());
        }


        odometry.stopT265();
    }
}



//
//        sleep(250);
//        moveTicksBack(440, 4000, 0.4, 20, this); // align back to wobble
//        sleep(250);
//        turnToV2(90, 4000, this);
//        sleep(250);
//
//
//        resetAngle();
//        odometry.setPose(0, 0, 0); //  reset for cycle spline
//        arm.moveTop();
//        crossBarrier(-50, 0, 0.6, 50000, 1, true, false); // move to wobble.
//
//        container.dumpRelease();
//        sleep(500);
//        container.dumpBlock();
////        crossBarrier(10, 18, 0.5, 5000, 0, false, false);
//
//        xTo( 50, 4000, 0.7, 3, this, true, true);