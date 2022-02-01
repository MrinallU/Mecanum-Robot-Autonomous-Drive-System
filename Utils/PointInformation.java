package org.firstinspires.ftc.teamcode.Utils;

public class PointInformation {
    public Point point;
    public double xAcc, yAcc, angAcc, timeout;

    public PointInformation(Point point, double xAcc, double yAcc, double angAcc, double timeout){
        this.point = point;
        this.xAcc = xAcc;
        this.yAcc = yAcc;
        this.angAcc = angAcc;
        this.timeout = timeout;
    }

    public PointInformation(Point point, double posAcc, double angAcc, double timeout){
        this(point, posAcc, posAcc, angAcc, timeout);
    }

    public PointInformation(Point point, double posAcc, double timeout){
        this(point, posAcc, 2, timeout);
    }

    public PointInformation(Point point, double timeout){
        this(point, 1, timeout);
    }

    public PointInformation(Point point){
        this(point, 5000);
    }

    @Override
    public boolean equals(Object o){
        if(o instanceof PointInformation){
            PointInformation oP = (PointInformation)o;

            if(oP.point.equals(point) && oP.xAcc == xAcc && oP.yAcc == yAcc && oP.angAcc == angAcc && oP.timeout == timeout){
                return true;
            }
        }

        return false;
    }
}