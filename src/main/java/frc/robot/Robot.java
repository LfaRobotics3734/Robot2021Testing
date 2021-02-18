// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  AHRS ahrs;
  private Joystick stick;

  VictorSP leftTopMotor;
  VictorSP leftBottomMotor;
  VictorSP rightTopMotor;
  VictorSP rightBottomMotor;
  SpeedControllerGroup m_left;
  SpeedControllerGroup m_right;
  DifferentialDrive m_drive;

  double distance;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftTopMotor = new VictorSP(0);
    leftBottomMotor = new VictorSP(1);
    rightTopMotor = new VictorSP(2);
    rightBottomMotor = new VictorSP(3);
    m_left = new SpeedControllerGroup(leftTopMotor, leftBottomMotor);
    m_right = new SpeedControllerGroup(rightTopMotor, rightBottomMotor);
    m_drive = new DifferentialDrive(m_left, m_right);

    ahrs = new AHRS(SPI.Port.kMXP); 
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    //get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("Vision");

    //get a reference to key in "datatable" called "Y"
    NetworkTableEntry orientationEntry = table.getEntry("orientation");

    //add an entry listener for changed values of "X", the lambda ("->" operator)
    //defines the code that should run when "X" changes
    table.addEntryListener("distance", (tab, key, entry, value, flags) -> {
        System.out.println("Distance changed value: " + value.getValue());
        distance = Double.parseDouble(value.getValue().toString());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    //add an entry listener for changed values of "Y", the lambda ("->" operator)
    //defines the code that should run when "Y" changes
    orientationEntry.addListener(event -> {
        System.out.println("Orientation 2 changed value: " + event.value.getValue());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    ahrs.zeroYaw();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
          
    System.out.println("IMU_CONNECTED:" +ahrs.isConnected());
    System.out.println("IS CALIBRATING: " + ahrs.isCalibrating() );
    System.out.println("YAW: " + ahrs.getYaw());
    System.out.println("Displacement_X: " + ahrs.getDisplacementX());
    System.out.println("Displacement_Y:" +ahrs.getDisplacementY());

    if (distance > 10) {
      m_drive.tankDrive(0.5, 0.5);
    }
    else {
      m_drive.tankDrive(0, 0);
    }
    
    

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
