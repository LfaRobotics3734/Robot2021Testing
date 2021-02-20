package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.SpeedControllerGroup;



/**
 * This is a demo program showing the use of the navX MXP to implement
 * a "rotate to angle" feature.
 *
 * This example will automatically rotate the robot to one of four
 * angles (0, 90, 180 and 270 degrees).
 *
 * This rotation can occur when the robot is still, but can also occur
 * when the robot is driving.  When using field-oriented control, this
 * will cause the robot to drive in a straight line, in whathever direction
 * is selected.
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */

public class Rotate extends TimedRobot implements PIDOutput {
    AHRS ahrs;

    Joystick stick;
    PIDController turnController;
    double rotateToAngleRate;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;

    // Channels for the wheels
    final static int frontLeftChannel	= 2;
    final static int rearLeftChannel	= 3;
    final static int frontRightChannel	= 1;
    final static int rearRightChannel	= 0;
        
    VictorSP leftTopMotor;
    VictorSP leftBottomMotor;
    VictorSP rightTopMotor;
    VictorSP rightBottomMotor;
    SpeedControllerGroup m_left;
    SpeedControllerGroup m_right;
    DifferentialDrive myRobot;

    @Override
    public void robotInit() {
    	leftTopMotor = new VictorSP(0);
        leftBottomMotor = new VictorSP(1);
        rightTopMotor = new VictorSP(2);
        rightBottomMotor = new VictorSP(3);
        m_left = new SpeedControllerGroup(leftTopMotor, leftBottomMotor);
        m_right = new SpeedControllerGroup(rightTopMotor, rightBottomMotor);
        myRobot = new DifferentialDrive(m_left, m_right);
        
        stick = new Joystick(0);
        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        
        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
        /* tuning of the Turn Controller's P, I and D coefficients.            */
        /* Typically, only the P value needs to be modified.                   */
        /* LiveWindow.addActuator("DriveSystem", "RotateController", turnController); */
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    @Override
    public void autonomousInit() {
        myRobot.setSafetyEnabled(false);
        myRobot.tankDrive(0.0,0.0);    // stop robot
        Timer.delay(2.0);		    			  //    for 2 seconds
        myRobot.tankDrive(0.0,0.0);
    }

    /**
     * Runs the motors with onnidirectional drive steering.
     * 
     * Implements Field-centric drive control.
     * 
     * Also implements "rotate to angle", where the angle
     * being rotated to is defined by one of four buttons.
     * 
     * Note that this "rotate to angle" approach can also 
     * be used while driving to implement "straight-line
     * driving".
     */
    @Override
    public void teleopPeriodic() {
        myRobot.setSafetyEnabled(true);
        boolean rotateToAngle = false;
        if ( stick.getRawButton(1)) {
            ahrs.reset();
        }
        if ( stick.getRawButton(2)) {
            turnController.setSetpoint(0.0f);
            rotateToAngle = true;
        } else if ( stick.getRawButton(3)) {
            turnController.setSetpoint(90.0f);
            rotateToAngle = true;
        } else if ( stick.getRawButton(4)) {
            turnController.setSetpoint(179.9f);
            rotateToAngle = true;
        } else if ( stick.getRawButton(5)) {
            turnController.setSetpoint(-90.0f);
            rotateToAngle = true;
        }
        double currentRotationRate;
        if ( rotateToAngle ) {
            turnController.enable();
            currentRotationRate = rotateToAngleRate;
        } else {
            turnController.disable();
            currentRotationRate = stick.getTwist();
        }
        try {
            /* Use the joystick X axis for lateral movement,          */
            /* Y axis for forward movement, and the current           */
            /* calculated rotation rate (or joystick Z axis),         */
            /* depending upon whether "rotate to angle" is active.    */
            myRobot.driveCartesian(stick.getX(), stick.getY(), 
                                    currentRotationRate, ahrs.getAngle());
        } catch( RuntimeException ex ) {
            DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
        }
        Timer.delay(0.005);		// wait for a motor update time
        
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }

    @Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
}