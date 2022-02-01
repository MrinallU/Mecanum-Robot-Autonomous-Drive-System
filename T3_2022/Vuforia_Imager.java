package org.firstinspires.ftc.teamcode.T3_2022;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;

@Disabled
@TeleOp(name="Vuforia-Imager-T3", group="T3")
public class Vuforia_Imager extends T3_Base {
    boolean x2P;
    boolean xL2P;
    int imgId = 1;

    @Override
    public void runOpMode() {

        init(0);
        T3_Camera camera = new T3_Camera(hardwareMap);
        initServos();
        sleep(2000);

        telemetry.addData("Camera Attempts", 0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        matchTime.reset();
        resetCache();

        while (opModeIsActive()) {
            resetCache();

            xL2P = x2P;
            x2P = gamepad2.x;
            if(!xL2P && x2P){
               camera.saveImage(imgId);
               ++imgId;
            }

            telemetry.addLine("Images Taken: " + imgId);
            telemetry.update();
        }
    }
}
