// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Suspension;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.util.function.BooleanSupplier;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final         CommandXboxController driver = new CommandXboxController(0);
  public final static XboxController m_Xbox = new XboxController(0);

  // final         CommandXboxController codriverXbox = new CommandXboxController(1);
  final CommandJoystick codriver = new CommandJoystick(2);
  public final static Joystick m_Joy = new Joystick(2);

  // private Elevator elevator = new Elevator(0, 0.05, 1, 0,0.05,0.5);
  private Elevator elevator = new Elevator();
  private final Intake intake = new Intake();
  // private final Suspension suspension = new Suspension(0, 0, 0)

  public static Limelight limelight = new Limelight();
  public static double varX=0;
  public static double varY=0;

  public static double yaw=0;
  public static double pitch=0;
  public static double roll=0;

  public static Rotation2d varAngle;
  public boolean zeroTag, var;
  
  public static double tempoDeAjuste=0.0;
  public static boolean alliance;
  public static boolean tog=false;
  private static int stepAlga=0;
  public static int stepClimber=0;
  public static int stepClimberReturn=0;

  private static double ajusteFino=1;

  public static int varreef2=0;

  public static int reef=1;

  public static boolean coletarAlgaAlto=false;
  public static boolean autoAlgaeTransport=false;

  SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  SwerveInputStream driveAngularVelocityBLUE = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driver.getLeftY() *driver.getRawAxis(3) * (-1)*ajusteFino,
    () -> driver.getLeftX() *driver.getRawAxis(3) * (-1)*ajusteFino)
    .withControllerRotationAxis(()-> driver.getRightX() * (-1))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
    // SwerveInputStream driveAngularVelocityBLUE = SwerveInputStream.of(drivebase.getSwerveDrive(),
    // () -> driver.getLeftY() *driver.getRawAxis(3) * (-1)*ajusteFino,
    // () -> driver.getLeftX() *driver.getRawAxis(3) * (-1)*ajusteFino)
    // .withControllerRotationAxis(()-> driver.getRightX() * (-1))
    // .deadband(OperatorConstants.DEADBAND)
    // .scaleTranslation(0.8)
    // .allianceRelativeControl(true);
  Command AnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocityBLUE);

  Command AlinhamentoAdvBlue = new AbsoluteDriveAdv(drivebase,
  ()->TyAling(),
  ()->TxAling(),
  ()->MathUtil.applyDeadband(driver.getRightX(), Constants.OperatorConstants.DEADBAND),
  ()->(codriver.button(2).getAsBoolean()),
  ()->codriver.button(3).getAsBoolean(),
  ()->codriver.button(6).getAsBoolean(),
  ()->codriver.button(5).getAsBoolean(),
  ()->codriver.button(7).getAsBoolean(),
  ()->codriver.button(4).getAsBoolean());

  Command AlinhamentoAdvRed = new AbsoluteDriveAdv(drivebase,
  ()->-TyAling(),
  ()->-TxAling(),
  ()->-MathUtil.applyDeadband(driver.getRightX(), Constants.OperatorConstants.DEADBAND),
  ()->codriver.button(5).getAsBoolean(),
  ()->codriver.button(6).getAsBoolean(),
  ()->codriver.button(3).getAsBoolean(),
  ()->codriver.button(2).getAsBoolean(),
  ()->codriver.button(4).getAsBoolean(),
  ()->codriver.button(7).getAsBoolean());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);   //// ESSSA MERDA PRECISA FICAR DEPOIS DO CONFIGURE BINDINGS
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    NamedCommands.registerCommand("Disparar", new InstantCommand(()-> Elevator.depositar()));
    NamedCommands.registerCommand("L1", new InstantCommand(()-> elevator.positionDeposit(Constants.Field.CORAL_L1)));
    NamedCommands.registerCommand("L2", new InstantCommand(()-> elevator.positionDeposit(Constants.Field.CORAL_L2)));
    NamedCommands.registerCommand("L3", new InstantCommand(()-> elevator.positionDeposit(Constants.Field.CORAL_L3)));
    NamedCommands.registerCommand("L4", new InstantCommand(()-> elevator.positionDeposit(Constants.Field.CORAL_L4)));
    NamedCommands.registerCommand("RepousoElevador", new InstantCommand(()-> elevator.positionDeposit(0)));
    NamedCommands.registerCommand("Intake", new InstantCommand(()-> intake.coletarCoral()));
    NamedCommands.registerCommand("RecolheIntake", new SequentialCommandGroup(
      new InstantCommand(()-> intake.recolheIntake()),
      new InstantCommand(()-> Intake.stopFunil())));

    NamedCommands.registerCommand("REEFA", new SequentialCommandGroup(     // Poderia ser paralelo
       new InstantCommand(()-> reef=1),
       Commands.runOnce(()-> limelight.setPipelineBoth(0))));

    NamedCommands.registerCommand("REEFB", new SequentialCommandGroup(
      new InstantCommand(()-> reef=2),
      Commands.runOnce(()-> limelight.setPipelineBoth(1))));

    NamedCommands.registerCommand("REEFC", new SequentialCommandGroup(
      new InstantCommand(()-> reef=3),
      Commands.runOnce(()-> limelight.setPipelineBoth(2))));

    NamedCommands.registerCommand("REEFD", new SequentialCommandGroup(
      new InstantCommand(()-> reef=4),
      Commands.runOnce(()-> limelight.setPipelineBoth(3))));

    NamedCommands.registerCommand("REEFE", new SequentialCommandGroup(
     new InstantCommand(()-> reef=5),
     Commands.runOnce(()-> limelight.setPipelineBoth(4))));

    NamedCommands.registerCommand("REEFF", new SequentialCommandGroup(
      new InstantCommand(()-> reef=6),
      Commands.runOnce(()-> limelight.setPipelineBoth(5))));

    NamedCommands.registerCommand("ColetarAlga", new SequentialCommandGroup(
      new InstantCommand(()-> coletarAlgaAlto=true),
      Commands.waitSeconds(2.5),
      new InstantCommand(()-> autoAlgaeTransport=true),
      new InstantCommand(()-> autoAlgaeTransport=false),
      new InstantCommand(()-> coletarAlgaAlto=false)));

    NamedCommands.registerCommand("ONCoralFunil", new SequentialCommandGroup(
    new InstantCommand(()-> Intake.autoFunil()),
    Commands.waitSeconds(1),
    new InstantCommand(()-> Intake.stopFunil()))); 
    
    driverCommands();
    codriverCommands();
    conditionalCommands();
  }

  void codriverCommands(){

    codriver.button(17).onTrue((new InstantCommand(()-> Elevator.depositar())));                            // DEPOSITAR CORAL

    codriver.button(16).onTrue(Commands.runOnce(()-> elevator.positionDeposit(0.3)));               // ELEVADOR REPOUSO
    codriver.button(8).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L1)));   // ELEVADOR L1
    codriver.button(12).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L1)));  // ELEVADOR L1
    codriver.button(9).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L2)));   // ELEVADOR L2
    codriver.button(13).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L2)));  // ELEVADOR L2
    codriver.button(10).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L3)));  // ELEVADOR L3
    codriver.button(14).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L3)));  // ELEVADOR L3
    codriver.button(11).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L4)));  // ELEVADOR L4
    codriver.button(15).onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L4)));  // ELEVADOR L4
    
    codriver.button(5).onTrue((Commands.runOnce(()-> limelight.setPipelineBoth(0)))); //  SET PIPEPLIEN FILTERS REEF 1
    codriver.button(6).onTrue((Commands.runOnce(()-> limelight.setPipelineBoth(1)))); //  SET PIPEPLIEN FILTERS REEF 2
    codriver.button(7).onTrue((Commands.runOnce(()-> limelight.setPipelineBoth(2)))); //  SET PIPEPLIEN FILTERS REEF 3
    codriver.button(2).onTrue((Commands.runOnce(()-> limelight.setPipelineBoth(3)))); //  SET PIPEPLIEN FILTERS REEF 4
    codriver.button(3).onTrue((Commands.runOnce(()-> limelight.setPipelineBoth(4)))); //  SET PIPEPLIEN FILTERS REEF 5
    codriver.button(4).onTrue((Commands.runOnce(()-> limelight.setPipelineBoth(5)))); //  SET PIPEPLIEN FILTERS REEF 6
  }

  void driverCommands(){

    driver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));    // ZERAR ORIENTAÇÃO DO ROBÔ
    driver.x().onTrue(Commands.runOnce(()-> Intake.stopFunil()));      // STOP FUNIL
    driver.a().onTrue(new SequentialCommandGroup(                      // DESCE ELEVADOR E COLETA NO CHÃO
      Commands.runOnce(()-> elevator.positionDeposit(0)),
      Commands.runOnce(()-> intake.coletarCoral())));

    driver.button(10).onTrue(new InstantCommand(elevator::coralStation));  // COLETAR CORAL STATION
    driver.b().onTrue((new InstantCommand(()-> Elevator.depositar())));           // DEPOSITAR CORAL
    
    
    // driver.povDown().onTrue(new SequentialCommandGroup(
    //   Commands.runOnce(()-> Intake.setL1(0.25, 0.4, 75)),  // Inclina, 
    //   Commands.waitSeconds(2),
    //   Commands.runOnce(()-> Elevator.setPositionOuTake(0.5, 1, 20))));

    driver.back().onTrue(Commands.runOnce(()-> elevator.positionDeposit(0.3)));                   // DESCANSO
    driver.povDown().onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L1)));   // ELEVADOR L1
    driver.povLeft().onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L2)));   // ELEVADOR L2
    driver.povRight().onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L3)));  // ELEVADOR L3
    driver.povUp().onTrue(Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L4)));     // ELEVADOR L4
  }

  void conditionalCommands(){

    /* DRIVER CONTROLLER */
    activateCommandOnCondition(()-> m_Xbox.getLeftTriggerAxis()<=0.3, new InstantCommand(()-> ajusteFino = 0.8));
    activateCommandOnCondition(()-> m_Xbox.getLeftTriggerAxis()>0.3, new InstantCommand(()-> ajusteFino = 1));

    BooleanSupplier isRightTriggerPressed = () -> driver.getRightTriggerAxis() > 0.2;
    BooleanSupplier isAdvBlue = () ->  alliance && (driver.getRightTriggerAxis() >= 0.08 && driver.getRightTriggerAxis() <= 0.2 && ((!m_Joy.getRawButton(8) || !m_Joy.getRawButton(12) || (!m_Joy.getRawButton(9) || !m_Joy.getRawButton(13)) || (!m_Joy.getRawButton(10) || !m_Joy.getRawButton(14)) || (!m_Joy.getRawButton(11) || !m_Joy.getRawButton(15)))));
    BooleanSupplier isAdvRed = () ->  !alliance && (driver.getRightTriggerAxis() >= 0.08 && driver.getRightTriggerAxis() <= 0.2 && ((!m_Joy.getRawButton(8) || !m_Joy.getRawButton(12) || (!m_Joy.getRawButton(9) || !m_Joy.getRawButton(13)) || (!m_Joy.getRawButton(10) || !m_Joy.getRawButton(14)) || (!m_Joy.getRawButton(11) || !m_Joy.getRawButton(15)))));
    activateCommandOnConditiondrivebase(isRightTriggerPressed, AnglularVelocity);         // Movimentação Padrão
    activateCommandOnConditiondrivebase(isAdvBlue, AlinhamentoAdvBlue);                   // Seta o modo de direção com alinhamento Blue Alliance
    activateCommandOnConditiondrivebase(isAdvRed, AlinhamentoAdvRed);                     // Seta o modo de direção com alinhamento Red Alliance

    BooleanSupplier protectElevador = () -> ((Math.abs(Constants.Robot_Chassy.pigeon.getPitch().getValueAsDouble()) > 3.5 || Math.abs(Constants.Robot_Chassy.pigeon.getRoll().getValueAsDouble()) > 4.5) && (Elevator.getEncoderElevator()>=Constants.Field.CORAL_L1));
    activateCommandOnCondition(protectElevador, Commands.runOnce(()-> elevator.positionDeposit(Constants.Field.CORAL_L1)));

    BooleanSupplier coletarAlgaL2 = () -> (driver.y().getAsBoolean() || coletarAlgaAlto) && (reef==2 || reef==4 || reef==6);
    BooleanSupplier coletarAlgaL3 = () -> (driver.y().getAsBoolean() || coletarAlgaAlto) && (reef==1 || reef==3 || reef==5);
    activateCommandOnCondition(coletarAlgaL2, Commands.runOnce(()-> elevator.coletarAlga(Constants.Field.ALGA_L2, -4.25)));   //-4.6 ambos
    activateCommandOnCondition(coletarAlgaL3, Commands.runOnce(()-> elevator.coletarAlga(Constants.Field.ALGA_L3, -4.25)));  // -4.2 tava legal tbm

    BooleanSupplier removerAlgaL2 = () -> driver.leftBumper().getAsBoolean() && (reef==2 || reef==4 || reef==6);
    BooleanSupplier removerAlgaL3 = () -> driver.leftBumper().getAsBoolean() && (reef==1 || reef==3 || reef==5);
    activateCommandOnCondition(removerAlgaL2, Commands.runOnce(()-> elevator.removerAlga(Constants.Field.ALGA_L2, -4)));  //-4.6 ambos
    activateCommandOnCondition(removerAlgaL3, Commands.runOnce(()-> elevator.removerAlga(Constants.Field.ALGA_L3, -4)));  // -4.2 tava legal tbm

    BooleanSupplier recolheIntake = ()-> driver.x().getAsBoolean() || Intake.getSensorIntake();
    activateCommandOnCondition(recolheIntake, Commands.runOnce(()-> intake.recolheIntake()));

    BooleanSupplier coletarAlgaDownDeposit = ()-> driver.rightBumper().getAsBoolean();
    activateCommandOnCondition(coletarAlgaDownDeposit, Commands.runOnce(()-> stepAlga++));
    activateCommandOnCondition(()-> stepAlga==1, Commands.runOnce(()-> Intake.coletarAlgastep1()));   // COLETAR ALGA NO CHÃO
    activateCommandOnCondition(()-> stepAlga==2, Commands.runOnce(()-> Intake.coletarAlgastep2()));   // PARAR DE RECOLHER A AGARRAR
    activateCommandOnCondition(()-> stepAlga==3, new SequentialCommandGroup(                          // DEPOSITAR ALGA
      Commands.runOnce(()-> Intake.depositarAlga()),
      Commands.waitSeconds(0.6),
      new InstantCommand(()-> stepAlga=0),
      new InstantCommand(()-> intake.recolheIntake())));

    // BooleanSupplier Climbar = ()-> (driver.back().getAsBoolean() );//&& Robot.elapsedTime >= 100
    // activateCommandOnCondition(Climbar, Commands.runOnce(()-> stepClimber++));
    // activateCommandOnCondition(()-> stepClimber==1, new SequentialCommandGroup(
    //   Commands.runOnce(()-> Suspension.suspensionStep1()),
    //   Commands.runOnce(()-> Intake.intakeClimber())));
    // activateCommandOnCondition(()-> stepClimber==2, new SequentialCommandGroup(
    //   Commands.runOnce(()-> Suspension.suspensionStep2()),
    //   new InstantCommand(()-> stepClimber=0))); /// COLOCAR O INTAKE PARA BAIXO AUTOMATICO
      
    // BooleanSupplier ClimberReturn = ()->   (driver.back().getAsBoolean() && Robot.elapsedTime <= 3);
    // activateCommandOnCondition(ClimberReturn,  Commands.runOnce(()-> stepClimberReturn++));
    // activateCommandOnCondition(()-> stepClimberReturn==1, Commands.runOnce(()-> Suspension.suspensionReturn()));
    // activateCommandOnCondition(()-> stepClimberReturn==2, new SequentialCommandGroup(
    //   Commands.runOnce(()-> Suspension.suspensionStep2()),
    //   new InstantCommand(()-> stepClimberReturn=0)));

    /* CODRIVER CONTROLLER */

    BooleanSupplier REEF_A  = ()-> codriver.button(5).getAsBoolean();
    BooleanSupplier REEF_B  = ()-> codriver.button(6).getAsBoolean();
    BooleanSupplier REEF_C  = ()-> codriver.button(7).getAsBoolean();
    BooleanSupplier REEF_D  = ()-> codriver.button(2).getAsBoolean();
    BooleanSupplier REEF_E  = ()-> codriver.button(3).getAsBoolean();
    BooleanSupplier REEF_F  = ()-> codriver.button(4).getAsBoolean();

    activateCommandOnCondition(REEF_A, new InstantCommand(()-> reef=1));
    activateCommandOnCondition(REEF_B, new InstantCommand(()-> reef=2));
    activateCommandOnCondition(REEF_C, new InstantCommand(()-> reef=3));
    activateCommandOnCondition(REEF_D, new InstantCommand(()-> reef=4));
    activateCommandOnCondition(REEF_E, new InstantCommand(()-> reef=5));
    activateCommandOnCondition(REEF_F, new InstantCommand(()-> reef=6));

    /* AUTOMATICO */
    BooleanSupplier isCoralIntakeDown = ()-> Constants.Robot_Chassy.sensIntake.get(); //&& Elevator.getEncoderElevator()<(2/4);
    BooleanSupplier isCoralIntakeUp = ()-> Constants.Robot_Chassy.sensIntake.get() && Elevator.getEncoderElevator()>1.5;
    BooleanSupplier isCoralIntakeNot = ()-> !Constants.Robot_Chassy.sensIntake.get() && Constants.Robot_Elevator.sensOuttake.get();

    activateCommandOnCondition(()-> !Constants.Robot_Chassy.sensIntake.get() && Constants.Robot_Elevator.sensOuttake.get(), Commands.runOnce(()-> Intake.stopFunil()));
    activateCommandOnCondition(isCoralIntakeDown, new InstantCommand(()-> Elevator.setPosiOuTake(-2.45)));    // 2.45 Garante que o coral estara fora de colisão
    activateCommandOnCondition(isCoralIntakeUp, new InstantCommand(()-> Elevator.setPosiOuTake(-0.35)));    // Garante que o coral estara fora de colisão
    activateCommandOnCondition(isCoralIntakeNot, new InstantCommand(()-> Elevator.setPosiOuTake(0.3)));                // Para o OutTake caso esteja fora de colisão o coral

    // BooleanSupplier isDeposit = ()-> ((Robot.elapsedTime >= 133 && Robot.elapsedTime < 135)  // DEPOSITA NO ULTIMO SEGUNDO
    // || (isDepositLimeLeft() || isDepositLimeRight()) && (inAngle(180) || inAngle(0) || inAngle(60) || inAngle(120) || inAngle(240) || inAngle(300))  // CONDIÇÃO DO ANGULO
    // && ((Elevator.getEncoderElevator() >= (Constants.Field.CORAL_L2-2) && Elevator.getEncoderElevator() <= (Constants.Field.CORAL_L2+2))                // LIME DEPOSITA L2
    // || ((Elevator.getEncoderElevator() >= (Constants.Field.CORAL_L3-2) && Elevator.getEncoderElevator() <= (Constants.Field.CORAL_L3+2)))));            // LIME DEPOSITA L3
    BooleanSupplier isDeposit = ()-> (Robot.elapsedTime >= 133 && Robot.elapsedTime < 135);
    activateCommandOnCondition(isDeposit, new InstantCommand(()-> Elevator.depositar()));


    
    // BooleanSupplier depositL1 = ()-> !Constants.Robot_Elevator.sensOuttake.get() && codriver.button(17).getAsBoolean();
    // activateCommandOnCondition(depositL1, new SequentialCommandGroup(
    //   Commands.runOnce(()-> Intake.setL1(0.25, 0.4, 35)),  // Inclina, 
    //   Commands.waitSeconds(1),
    //   Commands.runOnce(()-> Intake.setPosiColeta(-100)),
    //   Commands.waitSeconds(1),
    //   Commands.runOnce(()-> Intake.setL1(0.25, 0.4, 0))));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  Double TxAling() {
    double kP=0.12;     // 0.06
    double errorX=0;
    double xSetpointLime0=0, xSetpointLime1=0;
    double output=0;

    if (limelight.getTagId(0) < 0.2 && limelight.getTagId(1) < 0.2) {
        return 0.0;
    }

    if(m_Joy.getRawButton(8) || m_Joy.getRawButton(9) || m_Joy.getRawButton(10) || m_Joy.getRawButton(11)){
      if(limelight.getTx(0)!=0) errorX = xSetpointLime0 - limelight.getTx(0);
      if(limelight.getTx(1)!=0 && limelight.getTx(0)==0) errorX = (100 - limelight.getTx(1));
    }
    if(m_Joy.getRawButton(12) || m_Joy.getRawButton(13) || m_Joy.getRawButton(14) || m_Joy.getRawButton(15)){
      if(limelight.getTx(1)!=0) errorX = xSetpointLime1 - limelight.getTx(1);
      if(limelight.getTx(0)!=0 && limelight.getTx(1)==0) errorX = (-100 - limelight.getTx(0));
    }

    output = (m_Joy.getRawButton(8) || m_Joy.getRawButton(9) || m_Joy.getRawButton(10) || m_Joy.getRawButton(11)) ||  (m_Joy.getRawButton(12) || m_Joy.getRawButton(13) || m_Joy.getRawButton(14) || m_Joy.getRawButton(15))? kP * errorX : 0;
    double maxOut=0.4;

    if((limelight.getTx(0)>-2 && limelight.getTx(0)<2) || (limelight.getTx(1)>-2 && limelight.getTx(1)<2)){
      output = Math.max(-0.3, Math.min(0.3, output)); // bateria morta
    }
    else{
      output = Math.max(-maxOut, Math.min(maxOut, output)); // bateria morta
    }

    // if(reef==1) output*=1;
    // if(reef==4) output*=-1;
    // if(reef==2 || reef==3) output*=-0.95;
    // if(reef==5 || reef==6) output*=0.95;

    return output;
  }

  Double TyAling() {
    double kP=0.1;  //0.08
    double errorY=0;
    double ySetpointLime0=0, ySetpointLime1=0;  //ySetpointLime1=9.35  ySetpointLime0=10.4
    double output=0;

    if (limelight.getTagId(0) < 0.2 && limelight.getTagId(1) < 0.2) {
        return 0.0; // Retorna 0 se não for a tag desejada
    }

    if((m_Joy.getRawButton(8) || m_Joy.getRawButton(9) || m_Joy.getRawButton(10) || m_Joy.getRawButton(11)) && limelight.getTy(0)!=0){
      errorY = ySetpointLime0 - limelight.getTy(0);
    }
    if((m_Joy.getRawButton(12) || m_Joy.getRawButton(13) || m_Joy.getRawButton(14) || m_Joy.getRawButton(15)) && limelight.getTy(1)!=0){
      errorY = ySetpointLime1 - limelight.getTy(1);
    }

    output = (m_Joy.getRawButton(8) || m_Joy.getRawButton(9) || m_Joy.getRawButton(10) || m_Joy.getRawButton(11)) ||
     (m_Joy.getRawButton(12) || m_Joy.getRawButton(13) ||
     m_Joy.getRawButton(14) || m_Joy.getRawButton(15))? kP * errorY : 0;
    
    if(limelight.getTy(0)<-6 || limelight.getTy(1)<-8){
      output = Math.max(-0.43, Math.min(0.43, output));  //Bateria morta
    }
    else{
      output = Math.max(-0.32, Math.min(0.32, output));  //Bateria morta
    }
    
    // if(reef==1) output*=1;
    // if(reef==4) output*=-1;
    // if(reef==2 || reef==6) output*=-0.56;
    // if(reef==3 || reef==5) output*=0.56;

    return output;
  }

  private void activateCommandOnConditiondrivebase(BooleanSupplier condition, Command command) {
    new Trigger(condition).whileTrue(Commands.run(() -> {
        if (drivebase.getDefaultCommand() != null) {
            drivebase.getDefaultCommand().cancel(); // Interrompe o comando padrão atual
        }
        drivebase.setDefaultCommand(command); // Define o novo comando padrão
        drivebase.getDefaultCommand().schedule(); // Força a execução do novo comando
    }));
  }
  private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  
  private boolean inRangeLimeTx1(double min, double max){
    return (limelight.getTx(0)>=min && limelight.getTx(0)<=max  && limelight.getTx(0)!=0) ? true: false;
  }

  private boolean inRangeLimeTy1(double min, double max){
    return (limelight.getTy(0)>=min && limelight.getTy(0)<=max  && limelight.getTy(0)!=0) ? true: false;
  }

  private boolean inRangeLimeTx2(double min, double max){
    return (limelight.getTx(1)>=min && limelight.getTx(1)<=max  && limelight.getTx(1)!=0) ? true: false;
  }

  private boolean inRangeLimeTy2(double min, double max){
    return (limelight.getTy(1)>=min && limelight.getTy(1)<=max && limelight.getTy(1)!=0) ? true: false;
  }

  private boolean inAngle(double yawAngle){
    double Range=7;
    if((yawAngle-Range)<0){
      return yaw > (360-Range) || yaw < (yawAngle+Range) ? true:false;
    }
    else{
      return yaw > (yawAngle-Range) && yaw < (yawAngle+Range) ? true:false;
    }
  }

  private boolean isDepositLimeLeft(){
    return (inRangeLimeTx2(-5.64, 5.4) && inRangeLimeTy2(-0.14,6.95))
    || (inRangeLimeTx2(-4.31, 2.82) && inRangeLimeTy2(1.65,1.96))
    || (inRangeLimeTx2(-8.5, -2.33) && inRangeLimeTy2(3.6,3.7)) ? true: false;
  }

  private boolean isDepositLimeRight(){
    return (inRangeLimeTx1(-7.12, 4.3) && inRangeLimeTy1(-0.15,4.6))
    || (inRangeLimeTx1(-0.57, 4.1) && inRangeLimeTy1(2.78,2.95))
    || (inRangeLimeTx1(-5.51, 1.66) && inRangeLimeTy1(-0.1,0.4)) ? true: false;
  }

  public void setBreakElevator(){
    Elevator.configBreakElevator();
  }

  public static boolean elevatorManipulation(){
    return m_Joy.getRawButton(16) || m_Xbox.getBackButton() || m_Joy.getRawButton(8) || m_Joy.getRawButton(9)
    || m_Joy.getRawButton(10) || m_Joy.getRawButton(11) || m_Joy.getRawButton(12) || m_Joy.getRawButton(13)
    || m_Joy.getRawButton(14) || m_Joy.getRawButton(15);
  }
}