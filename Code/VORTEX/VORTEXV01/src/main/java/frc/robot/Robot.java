// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private double teleopStartTime;
  public static double elapsedTime;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    Constants.Robot_Elevator.mPrincipal.getEncoder().setPosition(0);
    Constants.Robot_Elevator.mOutInc.getEncoder().setPosition(0);
    Constants.Robot_Chassy.pigeon.setYaw(0);
    Constants.Robot_Suspension.mPrincipal.getEncoder().setPosition(0);
    Constants.Robot_Intake.mFunil.getEncoder().setPosition(0);
    Constants.Robot_Intake.mColeta.getEncoder().setPosition(0);
    Constants.Robot_Intake.mInclina.getEncoder().setPosition(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {

    CommandScheduler.getInstance().run();

    RobotContainer.yaw = Constants.Robot_Chassy.pigeon.getYaw().getValueAsDouble() % 360;
    RobotContainer.pitch = Constants.Robot_Chassy.pigeon.getPitch().getValueAsDouble();
    RobotContainer.roll = Constants.Robot_Chassy.pigeon.getRoll().getValueAsDouble();

    if (RobotContainer.yaw < 0) {
      RobotContainer.yaw += 360;
    }

    dashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
    m_robotContainer.setBreakElevator();
    
    RobotContainer.limelight.setLed(0, 1);
    RobotContainer.limelight.setLed(1, 1);

    
    Constants.Robot_Elevator.configElevator.inverted(false).idleMode(IdleMode.kCoast);
    Constants.Robot_Elevator.mPrincipal.configure(Constants.Robot_Elevator.configElevator, null, null);
  
    Constants.Robot_Elevator.configOutInc.inverted(false).idleMode(IdleMode.kCoast);
    Constants.Robot_Elevator.mOutInc.configure(Constants.Robot_Elevator.configElevator, null, null);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }

    m_robotContainer.setMotorBrake(true);
    RobotContainer.limelight.setLed(0, 0);
    RobotContainer.limelight.setLed(1, 0);

    teleopStartTime = Timer.getFPGATimestamp();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    elapsedTime = Timer.getFPGATimestamp() - teleopStartTime;
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }

  public void dashboard(){
    SmartDashboard.putNumber("Yaw", RobotContainer.yaw);
    SmartDashboard.putNumber("Pitch", RobotContainer.pitch);
    SmartDashboard.putNumber("Pitch", RobotContainer.roll);


    SmartDashboard.putNumber("Suspension", Constants.Robot_Suspension.mPrincipal.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator", Constants.Robot_Elevator.mPrincipal.getEncoder().getPosition());
    SmartDashboard.putNumber("Shotter", Constants.Robot_Elevator.mOutInc.getEncoder().getPosition());
    SmartDashboard.putNumber("Outtake", Constants.Robot_Elevator.mOuttake.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber", Constants.Robot_Suspension.mPrincipal.getEncoder().getPosition());
    SmartDashboard.putBoolean("SensorOutTake", Constants.Robot_Elevator.sensOuttake.get());
    SmartDashboard.putBoolean("SensorIntake", Constants.Robot_Chassy.sensIntake.get());
    SmartDashboard.putNumber("OutTake Velocity", Constants.Robot_Elevator.mOuttake.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake Velocity", Constants.Robot_Intake.mColeta.getEncoder().getVelocity());
    SmartDashboard.putNumber("Funil", Constants.Robot_Intake.mFunil.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake", Constants.Robot_Intake.mColeta.getEncoder().getPosition());
    SmartDashboard.putNumber("Inclina", Constants.Robot_Intake.mInclina.getEncoder().getPosition());
    SmartDashboard.putNumber("REEF",  RobotContainer.reef);
    SmartDashboard.putBoolean("toggle",RobotContainer.tog);
    SmartDashboard.putBoolean("AlgaAlto",RobotContainer.coletarAlgaAlto);

    SmartDashboard.putBoolean("Aliance", RobotContainer.alliance);
    SmartDashboard.putNumber("StepClimber", RobotContainer.stepClimber);
    SmartDashboard.putNumber("Tempo Teleop", elapsedTime);
    SmartDashboard.putNumber("FPGA", Timer.getFPGATimestamp());
  }
}
