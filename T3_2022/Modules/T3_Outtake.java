package org.firstinspires.ftc.teamcode.T3_2022.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Motor;

public class T3_Outtake {
    public Motor motor1;
    public T3_Container container;
    LinearOpMode opMode;

    public T3_Outtake(Motor motor1, T3_Container container, LinearOpMode opmode, double p){
        this.motor1 = motor1;
        this.container = container;
        this.opMode = opmode;
    }

    public void moveToPosition(double pos) {
        motor1.setTarget(pos);
        motor1.retMotorEx().setTargetPositionTolerance(3);
        motor1.toPosition();
        motor1.setPower(0.2);
    }

    public void moveTop(){
        container.sweepBlock();
        moveToPosition(970);
    }

    public void moveMid(){
        container.sweepBlock();
        moveToPosition(1150);
    }

    public void moveMidBluePrim(){
        container.sweepBlock();
        moveToPosition(1125);
    }


    public void moveMidRedSecond(){
        container.sweepBlock();
        moveToPosition(1140);
    }

    public void moveBottom(){
        container.sweepBlock();
        moveToPosition(1250);
    }

    public void moveBottomRedSecond(){
        container.sweepBlock();
        moveToPosition(1250);
    }

    public void moveBottomBlue(){
        container.sweepBlock();
        moveToPosition(1250);
    }

    public void moveBottomBlueSecondary(){
        container.sweepBlock();
        moveToPosition(1270);
    }

    public void sweepPos(){
        container.sweepBlock();
        moveToPosition(0);
        container.sweepRelease();
    }

    public void sweepPosReset(){
        container.sweepBlock();
        moveToPosition(0);
    }

    public void sweepPosTeleop(){
        container.dumpBlock();
        container.sweepBlock();
        moveToPosition(0);
        container.sweepRelease();
    }


    public void moveMidBlue(){
        container.sweepBlock();
        moveToPosition(1130);
    }



    public void autoInitPos(){
        moveToPosition(135);
    }

    public void dump(){
        container.dumpRelease();
    }
}
