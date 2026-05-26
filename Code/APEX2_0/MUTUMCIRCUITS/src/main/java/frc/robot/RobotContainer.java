// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    public double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // // o robo ira dirigir de acordo com o campo.
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public boolean HoodOK = false;
    private double timerValue = 2;

    final CommandXboxController Cmdriver = new CommandXboxController(0);
    public XboxController driver = new XboxController(0);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Hood mHood = new Hood();
    public Intake mIntake = new Intake();
    public Climber mClimber = new Climber();
    public SubSystemSIM mSim = new SubSystemSIM();

    private int intakectn = 0;
    private int climberMove = 0;

    public double colisionProtect = 1;
    public double elapsedTime = 0;

    private double netProtection = 19;
    boolean intakeViado = false;
        
    
    public RobotContainer() {

        configureBindings();

        NamedCommands.registerCommand("HOOD_SHOOT", Commands.sequence(
                Commands.runOnce(() -> HoodOK = true),
                Commands.runOnce(() -> Hood.setAlingAuto(true)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> Hood.setAlingAuto(false)),
                Commands.runOnce(() -> HoodOK = false),
                Commands.runOnce(() -> mHood.end())));
                
        NamedCommands.registerCommand("WAIT1",
            Commands.defer(
                () -> Commands.waitSeconds(timerValue),
                java.util.Set.of()
            ));

        /* ALTERADO */
        NamedCommands.registerCommand("HOOD_AIN", Commands.defer(() -> {
                boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
                double targetX = red ? 11.914 : 4.624;
                double targetY = 4.044;
            return drivetrain.alignToTargetCommand(targetX, targetY);},
             java.util.Set.of(drivetrain))
        );


        new EventTrigger("SET_SHOTTER").onTrue(Commands.runOnce(() -> mHood.setShooterRPM(3200)));
        new EventTrigger("INTAKE_ON").onTrue(Commands.runOnce(() -> intakectn=1));
        new EventTrigger("INTAKE_OFF").onTrue(Commands.runOnce(() -> Intake.setArticulated(0.025, 0, 0.4)));
        new EventTrigger("CLIMBER_UP").onTrue(setClimber(98));
        new EventTrigger("CLIMBER_DOWN").onTrue(setClimber(0));
        new EventTrigger("CLIMBER_PUSH").onTrue(setClimber(15));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);

    }

    private void configureBindings() {

        /* AUTONOMO - BEGIN */
        whileCommandOnCondition(() -> HoodOK, createHoodCommand());
        activateCommandOnCondition(()-> Hood.getIndexando() && HoodOK, auxIndexerAuto(1));
        activateCommandOnCondition(() -> mHood.isFinished(),  Commands.runOnce(() -> HoodOK = false));
        activateCommandOnCondition(()-> !HoodOK, endHood());
        /* AUTONOMO - END */

        /* SWERVE DRIVE - BEGIN */
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> { return drive
            .withVelocityX(-driver.getLeftY() * MaxSpeed * driver.getRightTriggerAxis() * drivetrain.getColision())
            .withVelocityY(-driver.getLeftX() * MaxSpeed * driver.getRightTriggerAxis() * drivetrain.getColision())
            .withRotationalRate(drivetrain.getOmegaCmd() * MaxSpeed);
        }));

        Cmdriver.start().onTrue(drivetrain.runOnce(() -> drivetrain.configAngleInit()));
        whileCommandOnCondition(()-> Hood.getAligned() && Hood.getIndexando() && driver.getRightBumperButton(), drivetrain.brakeX()); 
        /* SWERVE DRIVE - END */

        /* INTAKE - BEGIN */
        // Cmdriver.a().onTrue(Commands.runOnce(() -> mIntake.setIntakeRPM(2500)));
        activateCommandOnCondition(() -> driver.getLeftBumperButton(), new InstantCommand(() -> intakectn++));
        activateCommandOnCondition(()-> intakectn == 1, intakePreOn());
        
        activateCommandOnCondition(() -> intakectn == 1 && Intake.getArticulatedPosition() > netProtection, intakeOn(5000));
        activateCommandOnCondition(() -> intakectn >= 2, intakeOff());

        activateCommandOnCondition(()-> driver.getAButton() && Intake.getArticulatedPosition() > netProtection , dispenserOn());
        Cmdriver.a().onFalse(dispenserOff());

        activateCommandOnCondition(()-> (Math.abs(Intake.getIntakeVelocity()[0] * 60) <= 200 || Math.abs(Intake.getIntakeVelocity()[1] * 60) <= 200) && intakectn == 1, new SequentialCommandGroup(
            Commands.waitSeconds(2),
            Commands.runOnce(() -> intakectn = 2)
            .onlyIf(() -> (Math.abs(Intake.getIntakeVelocity()[0] * 60) <= 200 || Math.abs(Intake.getIntakeVelocity()[1] * 60) <= 200) && intakectn == 1)));
        /* INTAKE - END */

        /* ASSISTANCE INDEXER - BEGIN */
        activateCommandOnCondition(()-> Hood.getIndexando() && driver.getRightBumperButton() && driver.getXButton(), Commands.runOnce(() -> Intake.setArticulated(0.015, 0, 0.2))); // 01/05/26 08:48 - speed = 0.1
        activateCommandOnCondition(()-> Hood.getIndexando() && driver.getRightBumperButton() && driver.getXButton() && Intake.getArticulatedPosition() > netProtection, Commands.runOnce(() -> mIntake.setIntakeRPM(2500)));
        activateCommandOnCondition(()-> Hood.getIndexando() && driver.getRightBumperButton() && driver.getXButton() && Intake.getArticulatedPosition() <= netProtection, Commands.runOnce(() -> mIntake.setIntakeRPM(0)));

        Cmdriver.x().onFalse(indexerIntakeOff());
        activateCommandOnCondition(()-> Intake.getArticulatedPosition() > 2 && Intake.getArticulatedPosition() < 8 && climberMove == 1, indexerClimber());
        /* ASSISTANCE INDEXER - END */
    
        /* HOOD - BEGIN*/
        Cmdriver.rightBumper().whileTrue(createHoodCommand());
        Cmdriver.rightBumper().onFalse(endHood());

        Cmdriver.rightBumper().onTrue(Commands.runOnce(() -> climberMove = 1));
        /* HOOD - END */

        /* CLIMBER - BEGIN */
        Cmdriver.povUp().onTrue(setClimber(98));
        activateCommandOnCondition(()-> driver.getPOV() == 180 && !getTowerZone(), setClimber(0));
        activateCommandOnCondition(()-> driver.getPOV() == 180 && getTowerZone(), setClimber(15));
        
        Cmdriver.povLeft().whileTrue(followPath("D1.2"));
        Cmdriver.povRight().whileTrue(followPath("O1.2"));
        Cmdriver.back().whileTrue(followPath("TMID"));
        /* CLIMBER - END */

        /* STOP ALL MOTOR - BEGIN */
        Cmdriver.b().onTrue(stopAllMotors());
        activateCommandOnCondition(()-> Climber.getPosition() < 10 && driver.getBButton(), 
        Commands.runOnce(() -> Intake.setArticulated(0.1, 0, 0.5)));
        /* STOP ALL MOTOR - END */

        /* SIMULATION - BEGIN */
        activateCommandOnCondition(() -> intakectn == 1, new SequentialCommandGroup(
            Commands.runOnce(() -> mSim.setIntakeVelocity(4)),
            Commands.runOnce(() -> mSim.setSubArticula(22.394, 3)),
            Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 110, 5))));

        activateCommandOnCondition(() -> intakectn >= 2, Commands.runOnce(() -> mSim.setIntakeVelocity(0)));

        activateCommandOnCondition(()-> SubSystemSIM.getSubClimber() < 10 && driver.getBButton(), 
        Commands.runOnce(() -> mSim.setSubArticula(0, 3)));

        activateCommandOnCondition(()-> driver.getRightBumperButton() && driver.getXButton(), Commands.runOnce(() -> mSim.setSubArticula(0, 0.5)));
        Cmdriver.x().onFalse(Commands.runOnce(() -> mSim.setSubArticula(22.394, 3)));
        activateCommandOnCondition(()-> SubSystemSIM.getSubArticula() > 2 && SubSystemSIM.getSubArticula() < 8 && climberMove == 1,
            Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 0, 5)));

        Cmdriver.povUp().onTrue(Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 110, 5)));
        activateCommandOnCondition(()-> driver.getPOV() == 180 && !getTowerZone(), Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 0, 5)));
        activateCommandOnCondition(()-> driver.getPOV() == 180 && getTowerZone(), Commands.runOnce(() -> mSim.setSubClimber(drivetrain.getPose(), 15, 5)));
        /* SIMULATION - END */
    }

    /**
     * Executa um path desejado.
     * @param pathName String do path que deseja executar.
     * @return movimentos e ações presentes no path.
     */
    private Command followPath(String pathName) {
        return Commands.defer(() -> {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                return AutoBuilder.followPath(path);
            } catch (Exception e) {
                DriverStation.reportError("path Error: " + pathName, e.getStackTrace());
                return Commands.none();
            }
        }, java.util.Set.of(drivetrain));
    }

    public boolean getIntake(){
        return intakeViado;
    }

    /**
     * Move o climber para a posição desejada.
     * @param position Seta a posição desejada do climber.
     * @return Comando que move o climber
     */
    Command setClimber(double position){
        return Commands.runOnce(() -> mClimber.setPosition(drivetrain.getPose(), position, 1));
    }

    /**
     * Realiza o movimento suave do intake para dentro, ajudando o fluxo de disparos.
     * @param waitTimer Tempo de espera para ligar a inclinação do intake.
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command auxIndexerAuto(double waitTimer){
        return new SequentialCommandGroup(
            Commands.waitSeconds(waitTimer),
            Commands.runOnce(() -> Intake.setArticulated(0.01, 0, 0.4)),   //0.2
            Commands.runOnce(() -> mIntake.setIntakeRPM(2500)));
    }

    /**
     * Finaliza os comandos relacionados ao disparo e posiciona o intake para coleta novamente.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command endHood(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> mHood.end()),
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> Intake.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> intakectn = 0));
    }

    /**
     * Realiza a coleta de Fuels.
     * @param RPM RPM do intake.
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command intakePreOn(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Intake.setArticulated(0.1, 22.394, 0.5)),
            setClimber(98));
    }

    Command intakeOn(double RPM){
        return Commands.runOnce(() -> mIntake.setIntakeRPM(RPM));
    }
    /**
     * Desliga o intake
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command intakeOff(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            new InstantCommand(() -> intakectn = 0));
    }

    /**
     * Realiza sequencia de acionamentos para ejetar o fuel pelo intake.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command dispenserOn(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(-3500)),
            Commands.runOnce(() -> mHood.setBeltRPM(-4000)),
            Commands.runOnce(() -> mHood.setIndexRPM(-3000)),
            Commands.runOnce(() -> mHood.setFeedRPM(-1500)));
    }

    /**
     * Desliga o sistema de ejetar fuel.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command dispenserOff(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> mHood.setBeltRPM(0)),
            Commands.runOnce(() -> mHood.setIndexRPM(0)));
    }

    /**
     * Desliga o sistema de auxilio de fuls com intake.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command indexerIntakeOff(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)),
            Commands.runOnce(() -> Intake.setArticulated(0.1, 22.394, 0.5)),
            Commands.runOnce(() -> intakectn = 0));
    }

    /**
     * Posiciona o climber para baixo.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command indexerClimber(){
        return new SequentialCommandGroup(
            setClimber(0),
            Commands.runOnce(() -> climberMove = 0));
    }

    /**
     * Para todos os motores que conduzem a fuel.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    Command stopAllMotors(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> Hood.stopShooterSpeed()),
            Commands.runOnce(() -> Hood.stopIndexSpeed()),
            Commands.runOnce(() -> Hood.stopBelt()),
            Commands.runOnce(() -> mIntake.setIntakeRPM(0)));
    }

    /**
     * Realiza o path selecionado pelo autoChooser.
     * @param null
     * @return Autonomo selecionado.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.configAngleInit()),
                autoChooser.getSelected());
    }

    /**
     * Comando necessario para permitir o alinhamento do teleOperado no Autonomo.
     * @param null
     * @return sequencia de comandos necessarios para executar a ação.
     */
    private Command createHoodCommand() {
        return Commands.defer(() -> new Hood(), java.util.Set.of());
    }

    boolean getTowerZone(){
        double t_blueX = 1.074, t_blueY = 3.752;
        double t_redX = 15.479, t_redY = 4.323;

        return (drivetrain.getPose().getX() >= (t_blueX - 0.4) && drivetrain.getPose().getX() <= (t_blueX + 0.4) &&
            drivetrain.getPose().getY() >= (t_blueY - 1.15) && drivetrain.getPose().getY() <= (t_blueY + 1.15)) ||
            (drivetrain.getPose().getX() >= (t_redX - 0.4) && drivetrain.getPose().getX() <= (t_redX + 0.4) &&
            drivetrain.getPose().getY() >= (t_redY - 1.15) && drivetrain.getPose().getY() <= (t_redY + 1.15)) ? true : false;
    }


    /**
     * Configura o angulo inicial do robô, em função da aliança, sempre ligar com o intake para a frente, sentido meio da arena.
     * @param null
     */
    public void configInit(){
        drivetrain.configAngleInit();
    }

    public void getWait1Auto(double value){
        timerValue = value;;
    }

    /**
     * Ativa o comando alvo, quando a condição é verdadeira.
     * @param null
     */
    private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);
    }

    /**
     * Matém um comando ativo, enquanto a condição é verdade.
     * @param null
     */
    private void whileCommandOnCondition(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }
}