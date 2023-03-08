  // https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  public void updatePositionEncoderOdoBackup(){
    // apply mecnaum kinematic model (with wheel velocities [ticks per sec])
    double xV = (fLeftMotor.retMotorEx().getVelocity() + fRightMotor.retMotorEx().getVelocity()
            + bLeftMotor.retMotorEx().getVelocity() + bRightMotor.retMotorEx().getVelocity()) * 0.5;
    double yV =  (-fLeftMotor.retMotorEx().getVelocity() + fRightMotor.retMotorEx().getVelocity()
            + bLeftMotor.retMotorEx().getVelocity() - bRightMotor.retMotorEx().getVelocity()) * 0.5;

    // rotate the vector
    double nx = (xV*Math.cos(Math.toRadians(getAngle())))-(yV*Math.sin(Math.toRadians(getAngle())));
    double nY = (xV*Math.sin(Math.toRadians(getAngle())))+(yV*Math.cos(Math.toRadians(getAngle())));
    xV = nx; yV = nY;

    // integrate velocity over time
    curPose.yP+=(yV*(driveTime.seconds()-prevTime))/162.15; // <-- Tick to inch conversion factor
    curPose.xP+=(xV*(driveTime.seconds()-prevTime))/162.15;
    prevTime = driveTime.seconds();
  }

