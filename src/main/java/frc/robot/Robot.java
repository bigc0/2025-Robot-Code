// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private String m_autoSelected;
  private final WPI_VictorSPX LeftDrive = new WPI_VictorSPX(0);
  private final WPI_VictorSPX RightDrive = new WPI_VictorSPX(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(LeftDrive, RightDrive);
  private final XboxController m_controller = new XboxController(0);
  private final Joystick m_stick = new Joystick(1);
  private final Timer m_timer = new Timer();
  SparkMax elevator;
  SparkMax elevator2;
  SparkMax joint;
  SparkMax intake;
  SparkMax intake2;

public Robot() {

  //For Actual NEO Motors
  elevator = new SparkMax(3, MotorType.kBrushless);
  elevator2 = new SparkMax(4, MotorType.kBrushless);

  //For CIM Testing Motors
  // elevator = new SparkMax(3, MotorType.kBrushed);
  // elevator2 = new SparkMax(4, MotorType.kBrushed);

  //For joint on coral manipulator
  joint = new SparkMax(6, MotorType.kBrushed);

  //For coral intake
  intake = new SparkMax(5, MotorType.kBrushed);

  //Second coral intake
  intake2 = new SparkMax(7, MotorType.kBrushed);

  //Sets elevator to brake mode
  SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  elevatorConfig
    .idleMode(IdleMode.kBrake);

  //Sets elevator2 to follow elevator (CANID 3) and sets inverted to (true)
  //Also turns on brake mode
  SparkMaxConfig elevator2Config = new SparkMaxConfig();
  elevator2Config
    .follow(3, true)
    .idleMode(IdleMode.kBrake);

  elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  //Set intake2 to follow intake
  SparkMaxConfig intake2Config= new SparkMaxConfig();
  intake2Config
    .follow(5, false);

    intake2.configure(intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Turned on brake mode
    SparkMaxConfig jointConfig = new SparkMaxConfig();
    jointConfig
      .idleMode(IdleMode.kBrake);

    joint.configure(jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //For camera
    CameraServer.startAutomaticCapture();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    RightDrive.setInverted(true);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (m_timer.get() < 3) {
      // Drive forwards quarter speed for 3 seconds, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.25, 0.0, false);

      } else {
        m_robotDrive.stopMotor(); // stop robot
    }
    }
  
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_controller.getLeftY() * -1, m_controller.getRightX() * -1);

    
     //elevator up
    if (m_controller.getRawButton(5)) {
      elevator.set(0.25);
    }
    // elevator down
    else if (m_controller.getRawButton(6)) {
      elevator.set(-0.25);
    }
    else {
      elevator.set(0);
    }

    if (m_controller.getRawButton(1)) {
      intake.set(0.4);
    }
    else if (m_controller.getRawButton(2)) {
      intake.set(-0.4);
    }
    else {
      intake.set(0);
    }

    if (m_controller.getRawButton(3)) {
      joint.set(0.5);
    }
    else if (m_controller.getRawButton(4)) {
      joint.set(-0.5);
    }
    else {
      joint.set(0);
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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
