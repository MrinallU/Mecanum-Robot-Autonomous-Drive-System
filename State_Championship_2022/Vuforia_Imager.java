package org.firstinspires.ftc.teamcode.State_Championship_2022;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.State_Championship_2022.Modules.Camera;

@TeleOp(name="Vuforia-Imager", group="T3")
public class Vuforia_Imager extends Base {
    boolean x2P;
    boolean xL2P;
    int imgId = 1;

    @Override
    public void runOpMode() {

        init(0);
        Camera camera = new Camera(hardwareMap);
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
