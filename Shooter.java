package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Shooter {

    // two talons. 1 master, 1 slave
    // 1 spark max for hood for hood

    WPI_TalonFX masterShooterMotor = new WPI_TalonFX(Constants.masterShooterMotorIndex);
    WPI_TalonFX slaveShooterMotor = new WPI_TalonFX(Constants.slaveShooterMotorIndex);

    CANSparkMax hoodMotor = new CANSparkMax(Constants.hoodMotorIndex, MotorType.kBrushless);

    boolean shooting = false;
    boolean isTargetCenter = false; // x axis. y axis obviously can't be because of the angle

    final double maxHoodEncoders = 500; // could literally set position to angle // need to find through testing
    final double maxTurretEncoders = 500; // could literally set position to angle // need to find through testing

    final double saturationVoltage = 12.5;

    

    public double shooterIndexerEncoderCount = 200; // encoder count to go from under shooter to shooter.

    Timer shooterTimer = new Timer();

    boolean canShoot = false;

    public void shooterInit() { // might only needs this once in robot init

        slaveShooterMotor.follow(masterShooterMotor);  // want to start hood at the bottom. find encoder to angle ratio
        hoodMotor.setInverted(true);
        hoodMotor.getEncoder().setPositionConversionFactor(0.65222);
        hoodMotor.getEncoder().setPosition(10.5);
        masterShooterMotor.configVoltageCompSaturation(saturationVoltage);
        slaveShooterMotor.configVoltageCompSaturation(saturationVoltage);
        masterShooterMotor.enableVoltageCompensation(true);
        slaveShooterMotor.enableVoltageCompensation(true);
        canShoot = false;
        shooting = false;

        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); // this could be when shooting is true
        // otherwise it could just be a normal camera
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // could also be just when shooting is true
        // 3 or 0. 3 is force on. 0 is current pipeline mode

        // limelight pipeline = 1; // for vision tape
        //hoodMotor.getEncoder().setPositionConversionFactor() // find what it is to convert to degrees or radian
        
        // ideas:
        // set slave to update faster. frame period or something
        // set master shooter to have a conversion factor to convert into our in/s
    }

    final static double gravity = 386.103; // in/s^2
    final static double y = 104.016; // in // upper hub height by default. Should have button to switch between them
    final static double y0 = 22.5; // in
    final static double minimumVelocity = 267.7;
    final static double maximumVelocity = 405.5;
    final static double minimumAngle = 0.804;
    final static double maximumAngle = 1.387; // 1.352
    final static int velocityDecimals = 1;
    final static int angleDecimals = 3;

    final static double distanceFromTapeToCenterOfHub = 26.6875f; // in
    final static double distanceFromFenderToCenterOfHub = 33.875f;
    final static double distanceFromFenderToTape = distanceFromFenderToCenterOfHub - distanceFromTapeToCenterOfHub;

    final static double horizontalDistanceFromLimeLightToShooter = 5.375f; // in
    final static double horizontalDistanceFromShooterToFrontOfRobot = 7.25f;
    final static double horizontalDistanceFromLimelightToFrontOfRobot = horizontalDistanceFromShooterToFrontOfRobot - horizontalDistanceFromLimeLightToShooter;

    private static double cameraHeight = 21.0; // may need to account for difference between camera height and shooter
                                               // height
    private static double cameraAngle = 45.5; // make sure you account for radians vs degrees

    // Error from the limelight when reading the angle. It appears to be consistent
    private static double angleError = -1.23; // was 0.04

    static double calculatedVelocity = 0.0;
    static double calculatedAngle = 0.000;


    public static void testVelocity(double x) {

        double velocity = 0.00;
        for (double i = minimumVelocity; i <= maximumVelocity; i += (1.0 / Math.pow(10, velocityDecimals))) {
            if ((testAngle(i, x) > minimumAngle) && (testAngle(i, x) < maximumAngle)) {
                velocity = i;
                break;
            }
        }
        calculatedVelocity = velocity;
        calculatedAngle = testAngle(velocity, x);
    }

    public static double testAngle(double velocity, double x) {

        double angle = minimumAngle;

        double[] possibleAngles = new double[(int) ((maximumAngle * Math.pow(10, angleDecimals))
                - (minimumAngle * Math.pow(10, angleDecimals)) + 1)];
        for (int i = 0; i < possibleAngles.length; i++) {
            possibleAngles[i] = angle;
            angle = angle + (1 / Math.pow(10, angleDecimals));
        }

        for (double possibleAngle : possibleAngles) {
            double slope = slopeOfLine(velocity, possibleAngle, x);
            if (!Double.isNaN(slope)) {
                if (slope > 0.00) {
                    angle = possibleAngle;
                    break;
                }
            }
        }

        while (slopeOfLine(velocity, angle, x) > 0.000) {
            angle += (1.0 / Math.pow(10, angleDecimals));
        }
        return angle;
    }

    public static double slopeOfLine(double velocity, double possibleAngle, double x) {

        return ((y - x) / (((-1) * ((-1) * velocity * Math.sin(possibleAngle) - Math.sqrt(Math.pow(velocity, 2)
                * Math.pow(Math.sin(possibleAngle), 2) + ((2 * gravity) * (y0 - y)))) / gravity)
                - (x / (velocity * Math.cos(possibleAngle)))));
    }

    public static double getXDistanceFromCenterOfHub(double verticalAngle) {
        return ((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError))) + distanceFromTapeToCenterOfHub + horizontalDistanceFromLimeLightToShooter;
    }

    // public static double getXDistanceFromCenterOfHub(double verticalAngle) { // x distance from front of robot to fender. better for testing
    //     return ((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError))) - horizontalDistanceFromLimelightToFrontOfRobot - distanceFromFenderToTape; // ADD DISTANCE FROM TAPE TO CENTER
    // }



    public static double shooterWheelLinearVelocityToMotorVelocity(double shooterWheelLinearVelocity){
        return 2.1 * (shooterWheelLinearVelocity * (1.00/(4.00*Math.PI)) * (1.00/1.21) * 2048.00 * (1.00/10.00));
        // needs to be defined based on trial and error
        //return shooterWheelLinearVelocity; // calculate that initial v based on y = y0 + v*sin(theta)*t + 0.5*a*t^2
    }


    boolean setHoodYet = false;
    boolean centering = false;

    // 24 controls intake angle
    // 23 controls climber

    boolean goUp = false;
    boolean goDown = false;

    public void testingShooter(){

        System.out.println("angle error: " + angleError);

        // finding angle error for limelight for proper distance. find true distance first, then adjust
        if(Constants.stick.getRawButtonPressed(5)){
            angleError -= 0.05;
        } else if(Constants.stick.getRawButton(6)){
            angleError += 0.05;
        } else if(Constants.stick.getRawButton(3)){
            angleError -= 0.01;
        } else if(Constants.stick.getRawButtonPressed(4)){
            angleError += 0.01;
        }

        System.out.println("tv: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getNumber(0));
        
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getNumber(0).equals(1.0)){
            System.out.println("Found Target");
            System.out.println("Distance Calculated: " + getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.00)));
        } else {
            System.out.println("No Target Found");
            System.out.println("No Distance Calculated");
        }

        // System.out.println("Found Target: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false));
        // if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false)){
        //     System.out.println(getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.00)));
        // } else {
        //     System.out.println("No y angle");
        // }

        System.out.println("x angle: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(1000.0));
        System.out.println("y angle: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(1000.0));






        // testing shooting with calculated distance

        // if(Constants.stick.getRawButtonPressed(8)){
        //     if(y == 104.0f){
        //         y = 41.0f;
        //     }
        //     if(y == 41.0f){
        //         y = 104.0f;
        //     }
        // }

        if(Constants.stick.getRawButtonPressed(1)){
            shooting = !shooting;
            if(shooting){
                setHoodYet = false;
                shooterTimer.reset();
                shooterTimer.start();
                testVelocity(getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
                // should only be calculating velocity and angle when you have a real target. This would be after an if(tv && isTargetCenter)
                goUp = false;
                goDown = false;
            }
        }

        if(shooting){
            if(shooterTimer.get() > 0.5){
                masterShooterMotor.set(ControlMode.PercentOutput, (shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity) / 18850.6));

                if((hoodMotor.getEncoder().getPosition() < (90-Math.toDegrees(calculatedAngle))) && (goDown == false)){
                    goUp = true;
                }

                if(goUp){
                    hoodMotor.set(0.1);
                    if((hoodMotor.getEncoder().getPosition() > (90-Math.toDegrees(calculatedAngle)))){
                        hoodMotor.set(0.0);
                        setHoodYet = true;
                    }
                }

                if((hoodMotor.getEncoder().getPosition() > (90-Math.toDegrees(calculatedAngle))) && (goUp == false)){
                    goDown = true;
                }

                if(goDown){
                    hoodMotor.set(-0.1);
                    if((hoodMotor.getEncoder().getPosition() < (90-Math.toDegrees(calculatedAngle)))){
                        hoodMotor.set(0.0);
                        setHoodYet = true;
                    }
                }

                if((masterShooterMotor.getSelectedSensorVelocity() > shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity) - 50.0) && setHoodYet){
                    canShoot = true;
                } else {
                    canShoot = false;
                }
                if(canShoot){
                    Robot.intake.indexerMotor.set(-0.4);
                } else {
                    Robot.intake.indexerMotor.set(0.0);
                }

                // if((hoodMotor.getEncoder().getPosition() < (Math.toDegrees(calculatedAngle))) && !setHoodYet){
                //     hoodMotor.set(0.1);
                // } else {
                //     hoodMotor.set(0.0);
                //     hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
                //     setHoodYet = true;
                // }
                // if((hoodMotor.getEncoder().getPosition() > (Math.toDegrees(calculatedAngle))) && !setHoodYet){
                //     hoodMotor.set(-0.1);
                // } else {
                //     hoodMotor.set(0.0);
                //     hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
                //     setHoodYet = true;
                // }

                // if((masterShooterMotor.getSelectedSensorVelocity() > shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity) - 100.0) && setHoodYet){
                //     canShoot = true;
                // } else {
                //     canShoot = false;
                // }
                // if(canShoot){
                //     Robot.intake.indexerMotor.set(-0.4);
                // } else {
                //     Robot.intake.indexerMotor.set(0.0);
                // }
            }
        } else {
            masterShooterMotor.set(ControlMode.PercentOutput, 0.0);
            Robot.intake.indexerMotor.set(0.0);
        }







        /*// testing turning towards target. ONLY TURNS ONCE THEN STOPS WITHOUT RESETTING

        if(Constants.stick.getRawButtonPressed(2)){
            centering = !centering;
            if(centering){
                isTargetCenter = false; // have to press this again twice to reset it I think. We'll see
            }
        }

        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false) && centering){
            if((NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.00) < 0.0) && !isTargetCenter){
                Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.2);
                isTargetCenter = false;
                // check if current code works. if not, try:
                // if(ty > 0.0){
                //     targetCenter = true;
                // } // try same thing below

                // idea: have a boolean that becomes true after you make the initial determination of less than or greater than
                // then go to the move up or move down until it becomes the opposite
            } else {
                Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
                // PID?
                isTargetCenter = true;
            }

            if((NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.00) > 0.0) && !isTargetCenter){
                Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, -0.2);
                isTargetCenter = false;
            } else {
                Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
                // PID?
                isTargetCenter = true;
            }
        }*/


        
    }

    public void shooterTeleop() {

        /*
         * HAVE BUTTON TO SWITCH BETWEEN y's. AKA upper hub vs lower hub
         * notes:
         * robot needs to stop,
         * inactivate player controls,
         * turn itself and the turret until it finds the target
         * calculate the x distance to the target using limelight (angle + known height
         * of target)
         * calculate velocity and angle once
         * set motors and hood to go to those positions
         * verify they're at those positions
         * move the indexer wheel that number of encoder counts
         */

        // if(ColorSensor.getColor()){
        // Robot.intake.colorSensor2Activated = true;
        // }

        if ((Robot.intake.ballCount > 0) && Constants.stick.getRawButtonPressed(2) && !shooting) {
            shooting = true;
            Robot.drive.isDriverControlEnabled = false;
            Robot.drive.stopMotors();
            isTargetCenter = false;
        }

        // if (Robot.intake.ballCount == 0) {
        //     shooting = false;
        //     masterShooterMotor.set(0.0);
        //     Robot.drive.isDriverControlEnabled = true;
        // }

        if (shooting) {
            if (!NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false)) {
                Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.15); // arbitrary 0.3, just says how fast to turn in
                                                                        // order to find a target
            } else {
                if (!isTargetCenter) {
                    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
                            .getDouble(0.0) > 5.0) { // within 5 degrees
                        Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, -0.15);
                        /*if (turretMotor.getEncoder().getPosition() < maxTurretEncoders) {
                            turretMotor.set(-0.1);
                        }*/
                    } else if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx")
                            .getDouble(0.0) < -5.0) { // within 5 degrees
                        Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.15);
                    } else {
                        Robot.drive.stopMotors();
                        testVelocity((float) getXDistanceFromCenterOfHub((float) NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
                        isTargetCenter = true;
                    }
                } else {
                    //masterShooterMotor.set(ControlMode.Velocity, shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity)); // should be the speed according to some formula that sets it to the // initial velocity we need
                    masterShooterMotor.set(ControlMode.PercentOutput, (shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity) / 18850.6));
                    if((hoodMotor.getEncoder().getPosition() < (Math.toDegrees(calculatedAngle))) && !setHoodYet){
                        hoodMotor.set(0.1);
                        setHoodYet = false;
                    } else {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
                        setHoodYet = true;
                    }
                    if((hoodMotor.getEncoder().getPosition() > (Math.toDegrees(calculatedAngle))) && !setHoodYet){
                        hoodMotor.set(-0.1);
                        setHoodYet = false;
                    } else {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
                        setHoodYet = true;
                    }

                    //hoodMotor.set(Constants.quadraticPositionAndSpeed(0.05, 0.15, calculatedAngle, hoodMotor.getEncoder().getPosition())); // what is the best way to do this
                    if((masterShooterMotor.getSelectedSensorVelocity() > shooterWheelLinearVelocityToMotorVelocity(calculatedVelocity) - 300.0) && setHoodYet){
                        canShoot = true;
                        shooterTimer.reset();
                        shooterTimer.start();
                    } else {
                        canShoot = false;
                    }
                    if(canShoot){
                        Robot.intake.indexerMotor.set(0.4);
                    } else {
                        Robot.intake.indexerMotor.set(0.0);
                    }
                    if(shooterTimer.get() > (Robot.intake.ballCount*2.5)){
                        canShoot = false;
                        shooting = false;
                        Robot.intake.ballCount = 0;
                        masterShooterMotor.set(0.0);
                        Robot.drive.isDriverControlEnabled = true;
                    }
                }
            }
        }

        // here I have two options:
        // 1: use a timer and wait a certain amount of time (could be constant or based
        // on wanted motor speed)
        // 2: use an if statement and verify the speed is what we want. Contact with the
        // ball will slow down the motors. Does that matter?

        // if(timer is past certain point or motor is up to speed){
        // Robot.intake.indexerMotor.set(quadraticController(0.05,
        // Robot.intake.indexerMotorSpeed,
        // Robot.intake.indexerMotor.getEncoder().getPosition(), encoderCount));
        // if(Robot.intake.indexerMotor.getEncoder().getPosition() > encoderCount){
        // Robot.intake.indexerMotor.set(0.0);
        // Robot.intake.indexerMotor.getEncoder().setPosition(0.0);
        // Robot.intake.colorSensor2Activated = false;
        // }
        // }
    }

}