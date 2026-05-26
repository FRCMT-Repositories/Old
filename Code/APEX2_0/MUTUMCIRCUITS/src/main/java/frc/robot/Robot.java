// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private double timerTeleOp = 0;

    public static final Pigeon2 mPigeon2 = new Pigeon2(13);

    public static GenericEntry RPMShooter;
    public static GenericEntry KVShooter;
    public static GenericEntry setHmax;

    public static GenericEntry pipeline;
    public static GenericEntry auxiliar;
    public static GenericEntry auxiliar2;
    public static GenericEntry auxiliar3;
    public static GenericEntry wait1Timer;
    public static GenericEntry Driver;
    public static GenericEntry MIRAR;

    public static boolean isAuto = false;

    private Field2d field = new Field2d();

    private NetworkTable llFront = NetworkTableInstance.getDefault().getTable("limelight-front");
    private NetworkTable llLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
    private NetworkTable llRight = NetworkTableInstance.getDefault().getTable("limelight-right");

    public Robot() {
        m_robotContainer = new RobotContainer();
        
        RPMShooter = Shuffleboard.getTab("CONFIG").add("RPMShooter", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1, "orientation", "VERTICAL"))
        .withSize(10, 10)
        .getEntry();

        setHmax = Shuffleboard.getTab("CONFIG").add("Hmax", 1)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
            
        auxiliar = Shuffleboard.getTab("CONFIG").add("Auxiliar", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        auxiliar2 = Shuffleboard.getTab("CONFIG").add("Auxiliar2", 82)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        auxiliar3 = Shuffleboard.getTab("CONFIG").add("Auxiliar3", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        pipeline = Shuffleboard.getTab("CONFIG").add("setPipeline", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        wait1Timer = Shuffleboard.getTab("CONFIG").add("wait1Auto", 4)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        MIRAR = Shuffleboard.getTab("CONFIG").add("MIRAR", 1)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

        Logger.recordMetadata("ProjectName", "MeuRobo");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            // SIMULAÇÃO
            Logger.addDataReceiver(new WPILOGWriter("logs"));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        m_robotContainer.mIntake.setZeroArticulated();
        m_robotContainer.mHood.setZeroHood();
        m_robotContainer.mClimber.setZeroClimber();

        
        Pose2d robot = new Pose2d(2, 4, new Rotation2d(Math.PI));

        SmartDashboard.putData("FIELD", field);
        field.getObject("Robot").setPose(robot);

        m_robotContainer.configInit();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // elapsedTime = Timer.getFPGATimestamp() - teleopStartTime;

        double speedHood[] = Hood.getShooterVelocity();
        double speed[] = Intake.getIntakeVelocity();

        m_robotContainer.getWait1Auto(wait1Timer.getDouble(4));

        Logger.recordOutput("HOOD/v1", speedHood[0] * 60);
        Logger.recordOutput("HOOD/v2", speedHood[1] * 60);

        Logger.recordOutput("INTAKE/Coletor1Speed", speed[0] * 60);
        Logger.recordOutput("INTAKE/Coletor2Speed", speed[1] * 60);
        Logger.recordOutput("INTAKE/Position", Intake.getIntakePosition()[0]);
        Logger.recordOutput("INTAKE/Position", Intake.getIntakePosition()[1]);
        Logger.recordOutput("CURRENT/Intake", Intake.getIntakeCurrent()[0]);
        Logger.recordOutput("CURRENT/Intake", Intake.getIntakeCurrent()[1]);
        Logger.recordOutput("CURRENT/Hood", Hood.getHoodCurrent());
        Logger.recordOutput("CURRENT/Shotter1", Hood.getShooterCurrent()[0]);
        Logger.recordOutput("CURRENT/Shotter2", Hood.getShooterCurrent()[1]);
        Logger.recordOutput("CURRENT/Belt", Hood.getBeltCurrent());
        Logger.recordOutput("CURRENT/Index1", Hood.getIndexCurrent()[0]);
        Logger.recordOutput("CURRENT/Index2", Hood.getIndexCurrent()[1]);

        Logger.recordOutput("INTAKE/Articulation", Intake.getArticulatedPosition());
        Logger.recordOutput("HOOD/Position", Hood.getHoodPositon());
        Logger.recordOutput("HOOD/Index", Hood.getIndexVelocity());
        Logger.recordOutput("HOOD/Shooter", Hood.getShooterVelocity());
        Logger.recordOutput("Climber", Climber.getPosition());

        // Logger.recordOutput("HoodOK", m_robotContainer.getHoodOK());

        Logger.recordOutput("INDEXANDO", Hood.getIndexando());
        Logger.recordOutput("ALINHADO", Hood.getAligned());
        Logger.recordOutput("isEnd", Hood.isEnd());
        
        Logger.recordOutput("MaxSpeed", m_robotContainer.MaxSpeed);
        Logger.recordOutput("valuee", (Math.abs(Intake.getIntakeVelocity()[0] * 60) <= 1000 || Math.abs(Intake.getIntakeVelocity()[1] * 60) <= 1000));
        
        llFront.getEntry("pipeline").setNumber(pipeline.getDouble(0));
        llLeft.getEntry("pipeline").setNumber(pipeline.getDouble(0));
        llRight.getEntry("pipeline").setNumber(pipeline.getDouble(0));
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        isAuto = true;
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        isAuto = true;
    }

    @Override
    public void autonomousExit() {
        isAuto = true;
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        m_robotContainer.HoodOK = false;
        timerTeleOp = Timer.getFPGATimestamp();
        Hood.setAlingAuto(false);
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.elapsedTime = Timer.getFPGATimestamp() - timerTeleOp;
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    
}
