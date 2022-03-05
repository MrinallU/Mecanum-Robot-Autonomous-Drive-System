package org.firstinspires.ftc.teamcode.State_Championship_2022.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Motor;

public class Outtake {
    public Motor motor1;
    public Container container;
    LinearOpMode opMode;

    public Outtake(Motor motor1, Container container, LinearOpMode opmode, double p){
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
        moveToPosition(940);
    }

    public void moveTopSecondCycleBlue(){
        container.sweepBlock();
        moveToPosition(890);
    }

    public void moveMid(){
        container.sweepBlock();
        moveToPosition(1150);
    }

    public void moveMidBlueSecond(){
        container.sweepBlock();
        moveToPosition(1115);
    }

    public void moveMidBluePrim(){
        container.sweepBlock();
        moveToPosition(1135);
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
        moveToPosition(1240);
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
