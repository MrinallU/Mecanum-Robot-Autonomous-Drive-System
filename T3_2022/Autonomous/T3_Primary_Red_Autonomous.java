package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;

@Autonomous(name="T3_Primary_Red_Autonomous", group="Autonomous")
public class T3_Primary_Red_Autonomous extends T3_Base {
    int pos =  0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();



        odometry.updatePosition();

        pos = camera.readBarcode("redPrimary");
        if(pos == 0){
            telemetry.addData("Wobble Level: ", "Bottom");
            telemetry.addData("Shipping Element Placement: ", "☒ ☐ ☐");
            elementDiagram = "☒ ☐ ☐";
            telemetry.update();
        }else if(pos == 1){
            telemetry.addData("Wobble Level: ", "Middle");
            telemetry.addData("Shipping Element Placement: ", "☐ ☒ ☐");
            elementDiagram = "☐ ☒ ☐";
            telemetry.update();
        }else if(pos == 2){
            telemetry.addData("Wobble Level: ", "Top");
            telemetry.addData("Shipping Element Placement: ", "☐ ☐ ☒");
            elementDiagram = "☐ ☐ ☒";
            telemetry.update();
        }

        arm.moveToPosition(300);

        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();


        moveTicksBack(400, 4000, 0.5, 20,this);
        sleep(500);

        turnToV2(90, 4000, this);
        sleep(500);

        
        // todo, if time change the move back to odo
        if(pos == 2){
            arm.moveTop();
            sleep(2000);

            moveTicksFront(400, 4000, 0.5, 20, this);
            sleep(500);

            turnToV2(90, 4000, this);
            sleep(250);

            arm.dump();
            sleep(500);
            moveTicksBack(430, 4000, 0.5, 20, this);
        }else if(pos == 1){
            arm.moveMid();
            sleep(2000);

            moveTicksFront(350, 4000, 0.5, 20, this);
            sleep(500);

            turnToV2(90, 4000, this);
            sleep(250);

            arm.dump();
            sleep(500);

            moveTicksBack(200, 4000, 0.5, 20, this);
        }else{
            arm.moveBottom();
            sleep(2000);

            moveTicksFront(250, 4000, 0.5, 20, this);
            sleep(500);

            turnToV2(90, 4000, this);
            sleep(250);

            arm.dump();
            sleep(500);

            moveTicksBack(200, 4000, 0.5, 20, this);
        }

        // normalize move back to odo pos

        sleep(500);
        arm.container.dumpBlock();
        arm.moveToPosition(300);
        sleep(1000);

        moveTicksBack(700, 4000, 0.5, 20,this);
        sleep(500);

        turnToV2(-180, 4000, this);
        sleep(500);

        moveTicksBack(570, 1500, 0.1, 20, this);
        sleep(500);

        startCarousel();
        sleep(3000);
        stopCarousel();

        moveTicksFront(700, 4000, 0.3, 20,this);
        sleep(500);

        while(opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            telemetry.addLine("cam pos " + odometry.outStr);
            telemetry.addLine("imu angle " + getAngle());
        }

        odometry.stopT265();
    }
}
