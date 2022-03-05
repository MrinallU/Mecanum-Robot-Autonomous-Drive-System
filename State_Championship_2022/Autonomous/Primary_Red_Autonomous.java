package org.firstinspires.ftc.teamcode.State_Championship_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.State_Championship_2022.Modules.Camera;
import org.firstinspires.ftc.teamcode.State_Championship_2022.Base;

@Autonomous(name="Primary_Red_Autonomous", group="State")
public class Primary_Red_Autonomous extends Base {
    int pos =  1;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        Camera camera = new Camera(hardwareMap);
        initOdometry();

        sleep(3000);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        try {
            pos = camera.readBarcode("redPrimary");
        }catch (Exception e){

            pos = 1;
        }

        if(pos == 0){
            telemetry.addData("Wobble Level: ", "TOP");
            telemetry.addData("Shipping Element Placement: ", "☒ ☐ ☐");
            elementDiagram = "☒ ☐ ☐";
            telemetry.update();
        }else if(pos == 1){
            telemetry.addData("Wobble Level: ", "Middle");
            telemetry.addData("Shipping Element Placement: ", "☐ ☒ ☐");
            elementDiagram = "☐ ☒ ☐";
            telemetry.update();
        }else if(pos == 2){
            telemetry.addData("Wobble Level: ", "Bottom");
            telemetry.addData("Shipping Element Placement: ", "☐ ☐ ☒");
            elementDiagram = "☐ ☐ ☒";
            telemetry.update();
        }

        arm.moveToPosition(300);

        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.update();


        moveTicksBack(300, 4000, 0.5, 20,this);
        sleep(500);

        turnToV2(90, 4000, this);
        sleep(500);

        
        // todo, if time change the move back to odo
        if(pos == 0){
            arm.moveTop();
            sleep(2000);

            moveTicksFront(400, 4000, 0.5, 20, this);
            sleep(500);

            turnToV2(90, 4000, this);
            sleep(250);

            arm.dump();
            sleep(500);
//            moveTicksBack(430, 4000, 0.5, 20, this);
        }else if(pos == 1){
            arm.moveMid();
            sleep(2000);

            moveTicksFront(350, 4000, 0.5, 20, this);
            sleep(500);

            turnToV2(90, 4000, this);
            sleep(250);

            arm.dump();
            sleep(500);

//            moveTicksBack(200, 4000, 0.5, 20, this);
        }else{
            arm.moveBottom();
            sleep(2000);

            moveTicksFront(250, 4000, 0.5, 20, this);
            sleep(500);

            turnToV2(90, 4000, this);
            sleep(250);

            arm.dump();
            sleep(500);

//            moveTicksBack(200, 4000, 0.5, 20, this);
        }

        yTo(-21, 4000, 0.3, 1, this, true);

        sleep(500);
        arm.container.dumpBlock();
        arm.moveToPosition(300);
        sleep(1000);


        turnToV2(-175, 4000, this);
        sleep(500);

        moveTicksBack(250, 1500, 0.05, 20, this);
        sleep(500);
        moveTicksBack(270, 500, 0.05, 20, this);

        startCarousel();
        sleep(3000);
        stopCarousel();

        moveTicksFront(700, 4000, 0.3, 20,this);
        sleep(500);

        while(opModeIsActive()){
            resetCache();
            telemetry.addLine("imu angle " + getAngle());
        }

    }
}
