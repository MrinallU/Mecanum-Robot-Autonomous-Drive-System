package org.firstinspires.ftc.teamcode.Mecanum_2.TeleOP;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mecanum_2.Base;
import org.firstinspires.ftc.teamcode.Utils.Angle;

public class Test_TeleOp extends Base {

    // TeleOp Variables
    ElapsedTime matchTime;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(0, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        matchTime.reset();
        resetCache();
        dt.updateOdometry();

        while (opModeIsActive()) {
            // Updates
            resetCache();
            dt.updateOdometry();

            // Reset Angle
            currAngle = dt.getAngle();
            if (gamepad1.x) {
                targetAngle = -currAngle - 180;
            }

            // Change Drive Mode
            yLP = yP;
            yP = gamepad1.y;
            if (!yLP && yP) {
                basicDrive = !basicDrive;
            }

            // Drive
            slowDrive = gamepad1.left_bumper;
            fastDrive = gamepad1.left_trigger > 0.05;
            drive = floor(gamepad1.right_stick_y) * multiplier;
            strafe = floor(-gamepad1.right_stick_x) * multiplier;
            turn = turnFloor(gamepad1.left_stick_x) * multiplier;
            computeDrivePowers(gamepad1);


            // Display Values
            telemetry.addData("Drive Type", driveType);
            telemetry.addData("Odometry Info", odometry.outStr);
            telemetry.update();
        }
    }
}
