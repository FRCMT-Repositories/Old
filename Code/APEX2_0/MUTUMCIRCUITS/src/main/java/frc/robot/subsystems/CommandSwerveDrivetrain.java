package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix6.swerve.SwerveModule;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Robot;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private XboxController m_Control = new XboxController(0);

    String frontCAM = "limelight-front";
    // String leftCAM  = "limelight-left";
    // String rightCAM = "limelight-right";

    PIDController headingPID = new PIDController(0.028, 0.0001, 0.001);

    private double fixedAngle = 0;

    private double colisionProtect = 1;
    private static double OmegaCmd = 0;
    private boolean ctrInit = true;

    public boolean shotOk = false;
    public boolean alinhoAuto = false;

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public Field2d field = new Field2d();

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SwerveDriveBrake brakeX = new SwerveRequest.SwerveDriveBrake();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) { return run(() -> this.setControl(request.get())); }


    /**
     * Ativa o alinhamento em X das rodas.
     * @param null
     */
    public Command brakeX() {
        return applyRequest(() -> brakeX);
    }

    @Override
    public void periodic() {

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation   
                        : kBlueAlliancePerspectiveRotation  
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        var mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue(frontCAM);
        // var mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue(leftCAM);
        // var mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue(rightCAM);

        if(ctrInit){
            fixedAngle = isRedAlliance() ? 0 : 180;

            headingPID.enableContinuousInput(-180, 180);
            headingPID.setTolerance(2);

            // if(!Robot.isAuto){
            //     if(!isRedAlliance()) this.resetPose(new Pose2d(2, 4, new Rotation2d(Math.PI)));
            //     else this.resetPose(new Pose2d(13.5, 4, new Rotation2d(0)));
            // }

            ctrInit = false;
        }

        double YawRaw = this.getPigeon2().getYaw().getValueAsDouble();
        double YawReal = YawRaw;
        double YawWrapping = MathUtil.inputModulus(YawReal, -180, 180);

        double omega = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

        if (megaTagUpdateOdometry(mt2Front, 4, 30, omega)) {
            double xyStdDev = mt2Front.tagCount > 2 ? 0.1 : 0.1 + (mt2Front.avgTagDist * 0.2);
            // Pose2d visionPose = new Pose2d(mt2Front.pose.getTranslation(), new Rotation2d(Math.toRadians(YawWrapping)));
            Pose2d visionPose = new Pose2d(mt2Front.pose.getTranslation(), new Rotation2d(mt2Front.pose.getRotation().getRadians()));

            addVisionMeasurement(visionPose, mt2Front.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 999999.0));
            // updateLimelightAngle(frontCAM, YawWrapping);
            Logger.recordOutput("VISION/Front", mt2Front.pose);
        }

        Logger.recordOutput("VISION/HEADING", getHeading());

        boolean inZoneAlliance = (isRedAlliance() && (getPose().getX() >= 12.41)) || (!isRedAlliance() && (getPose().getX() <= 4.3));

        // if (megaTagUpdateOdometry(mt2Left, 4, 30, omega) && !m_Control.getRightBumperButton() && inZoneAlliance) {
        //     double xyStdDev = mt2Left.tagCount > 2 ? 0.1 : 0.1 + (mt2Left.avgTagDist * 0.2);
        //     Pose2d visionPose = new Pose2d(mt2Left.pose.getTranslation(), new Rotation2d(Math.toRadians(YawWrapping)));

        //     addVisionMeasurement(visionPose, mt2Left.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 999999.0));
        //     // updateLimelightAngle(leftCAM, YawWrapping);     /* ALTERADO */
        //     Logger.recordOutput("VISION/Left", mt2Left.pose);
        // }

        // if (megaTagUpdateOdometry(mt2Right, 4, 30, omega) && !m_Control.getRightBumperButton() && inZoneAlliance) {
        //     double xyStdDev = mt2Right.tagCount > 2 ? 0.1 : 0.1 + (mt2Right.avgTagDist * 0.2);
        //     Pose2d visionPose = new Pose2d(mt2Right.pose.getTranslation(), new Rotation2d(Math.toRadians(YawWrapping)));

        //     addVisionMeasurement(visionPose, mt2Right.timestampSeconds, edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, 999999.0));
        //     // updateLimelightAngle(rightCAM, YawWrapping);   /* ALTERADO */
        //     Logger.recordOutput("VISION/Right", mt2Right.pose);
        // }

        Pose2d currentPose = this.getState().Pose;
        Rotation2d heading = currentPose.getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, heading);

        double speedX = fieldSpeeds.vxMetersPerSecond;

        if(Climber.getPosition() >= 1){
            if(currentPose.getX() >= 4.628 - 1 && currentPose.getX() <= 4.628 + 1){
                if((currentPose.getY() >= 0 && currentPose.getY() <= 1.4)){
                    if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else if((currentPose.getY() >= 6.393 && currentPose.getY() <= 8.2)){
                    if((currentPose.getX() - 4.628) < 0 && speedX > 0.5 || (currentPose.getX() - 4.628) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 4.628) < 0 && speedX < -0.5 || (currentPose.getX() - 4.628) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else{
                    colisionProtect = 1;
                }
            }
            else if(currentPose.getX() >= 11.927 - 1 && currentPose.getX() <= 11.927 + 1){
                if((currentPose.getY() >= 0 && currentPose.getY() <= 1.4)){
                    if((currentPose.getX() - 11.927) < 0 && speedX > 0.5 || (currentPose.getX() - 11.927) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 11.927) < 0 && speedX < -0.5 || (currentPose.getX() - 11.927) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else if((currentPose.getY() >= 6.393 && currentPose.getY() <= 8.2)){
                    if((currentPose.getX() - 11.927) < 0 && speedX > 0.5 || (currentPose.getX() - 11.927) > 0 && speedX < -0.5) colisionProtect = 0.25; 
                    if((currentPose.getX() - 11.927) < 0 && speedX < -0.5 || (currentPose.getX() - 11.927) > 0 && speedX > 0.5) colisionProtect = 1;
                }
                else{
                    colisionProtect = 1;
                }
            }
            else colisionProtect = 1;
        }
        else{
            colisionProtect = 1;
        }

        if(m_Control.getYButton()){
            double currentDeg = getHeading().getDegrees();
            double targetDeg = 0;

            if(currentDeg > 0 && currentDeg <= 90) targetDeg = 45;
            else if(currentDeg > 90 && currentDeg <= 180) targetDeg = 135;
            else if(currentDeg >= -90 && currentDeg < 0) targetDeg = -45;
            else if(currentDeg >= -180 && currentDeg < -90) targetDeg = -135;

            Rotation2d targetAngle = Rotation2d.fromDegrees(targetDeg);
            OmegaCmd = headingPID.calculate(getHeading().getDegrees(), targetAngle.getDegrees());
            OmegaCmd = MathUtil.clamp(OmegaCmd, -1, 1);
            fixedAngle = YawWrapping;
        }
        else if(Math.abs(m_Control.getRightX()) < 0.1){
            if(m_Control.getRightBumperButton()){
                boolean mode = false;

                OmegaCmd = Robot.MIRAR.getDouble(0) == 1 ? Hood.getOmega() : -m_Control.getRightX();
                mode = Robot.MIRAR.getDouble(0) == 1 ? false : true;

                Hood.setAlingAuto(mode);

                fixedAngle = YawWrapping;
            }
            else{
                Rotation2d targetAngle = Rotation2d.fromDegrees(fixedAngle);
                OmegaCmd = headingPID.calculate(fixedAngle, targetAngle.getDegrees());
                OmegaCmd = MathUtil.clamp(OmegaCmd, -1, 1);
            }
        }
        else{
            if(m_Control.getRightBumperButton()){
                OmegaCmd = Hood.getOmega();
            }
            fixedAngle = YawWrapping;
            OmegaCmd = -m_Control.getRightX();
        }

        Hood.setRobotPose(currentPose);

        Logger.recordOutput("shotOk", shotOk);
        Logger.recordOutput("OmegaCmd", OmegaCmd);

        Logger.recordOutput("PIGEON/Raw", YawWrapping);
        Logger.recordOutput("PIGEON/Liar", MathUtil.inputModulus(YawRaw, -180, 180));
        Logger.recordOutput("PIGEON/Real", YawReal);
        Logger.recordOutput("PIGEON/fixed", fixedAngle);

        Logger.recordOutput("ODOMETRIA", currentPose);

        Logger.recordOutput("POSE/Odometry/Real", new double[] {currentPose.getX(), currentPose.getY(), Math.toRadians(YawWrapping)});
        Logger.recordOutput("POSE/Odometry/Estimate", new double[] {currentPose.getX(), currentPose.getY(), getPose().getRotation().getRadians()});

        Logger.recordOutput("VISION/mt2Front", mt2Front != null ? mt2Front.pose.getX() : 0.0);
        // Logger.recordOutput("VISION/mt2Left", mt2Left != null ? mt2Left.pose.getX() : 0.0);
        // Logger.recordOutput("VISION/mt2Right", mt2Right != null ? mt2Right.pose.getX() : 0.0);

        Pose2d robot = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(getPose().getRotation().getRadians()));
        SmartDashboard.putData("FIELD", field);
        field.getObject("Robot").setPose(robot);
    }

    public boolean isValid(LimelightHelpers.PoseEstimate est) {
    return est != null
        && est.tagCount > 1
        && est.avgTagDist < 4.0;
    }

    public boolean megaTagUpdateOdometry(LimelightHelpers.PoseEstimate mt2, double maxDistanceTAG, double maxRotation, double omega) {
        return mt2 != null && mt2.tagCount > 1 && mt2.avgTagDist < maxDistanceTAG && omega < maxRotation ? true : false;
    }

    public void updateLimelightAngle(String limelightName, double newAngle) {
        LimelightHelpers.SetRobotOrientation(limelightName, newAngle, 0, 0, 0, 0, 0);
    }

    public void zeroGyroPigeon() {
        this.getPigeon2().setYaw(0);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public double getColision(){
        return colisionProtect;
    }

    public double getOmegaCmd(){
        return OmegaCmd;
    }

    // Configura o PathPlanner
    private void configurePathPlanner() {
        try {
            //pega o arquivo .json com as configs do robo.
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose, 
                () -> getState().Speeds, 
                (speeds, feedforwards) -> setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(new PIDConstants(5, 0.0, 0.0), new PIDConstants(5, 0.0, 0.0)),
                config, 
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
                this
            ); 
        } catch (Exception e) {
            DriverStation.reportError("Falha ao configurar PathPlanner: " + e.getMessage(), true);
        }
    }

    public Command alignToTargetCommand(double targetX, double targetY) {
        SwerveRequest.FieldCentric holdAndRotate = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

        return run(() -> {
            Pose2d pose = getPose();
            Translation2d robot = pose.getTranslation();
            Rotation2d heading = pose.getRotation();

            Translation2d pose_Target = new Translation2d(targetX, targetY);
            Translation2d offsetHood = new Translation2d(0.19719, 0);
            Translation2d poseHood = robot.plus(offsetHood.rotateBy(heading));

            Translation2d angleHoodHub = pose_Target.minus(poseHood);
            Rotation2d targetAngleHood = angleHoodHub.getAngle();

            OmegaCmd = headingPID.calculate(heading.getDegrees(), targetAngleHood.getDegrees());
            OmegaCmd = MathUtil.clamp(OmegaCmd, -0.7, 0.7);

            if(Math.abs(headingPID.getError()) < 6) {OmegaCmd = 0;}

            fixedAngle = heading.getDegrees(); /* ALTERADO */

            setControl(holdAndRotate
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(OmegaCmd * 5.12));
        }).until(() -> {
            Pose2d pose = getPose();
            Rotation2d heading = pose.getRotation();

            Rotation2d desiredAngle =
                new Translation2d(targetX, targetY)
                    .minus(pose.getTranslation())
                    .getAngle();

            double error = MathUtil.angleModulus(desiredAngle.minus(heading).getRadians());
            
            alinhoAuto = Math.abs(error) < 6 ? true : false;
            Hood.setAlingAuto(alinhoAuto);

            if(Math.abs(error) < 6) OmegaCmd = 0;
            fixedAngle = heading.getDegrees(); /* ALTERADO */

            return Math.abs(error) < 6;
        });
    }

    public boolean getAlinhou(){
        return alinhoAuto;
    }

    /**
    * Metodo utilizado para atualizar a odometria do robo de forma precisa quando o mesmo se encontra parado.
    * @param mt2 MegaTAG2 relacionado a camera alvo.
    */
    public void odometryUpdateAutonomo() {
        double velocidadeGiro = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

        var mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue(frontCAM);
        // var mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue(leftCAM);
        // var mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue(rightCAM);

        if (mt2Front != null && mt2Front.tagCount > 0 && mt2Front.avgTagDist < 3.5 && velocidadeGiro < 45.0) {
            Pose2d currentPose = this.getState().Pose;
            Pose2d newPose = new Pose2d(mt2Front.pose.getTranslation(), currentPose.getRotation());
            this.resetPose(newPose);
            System.out.println("[VISÃO FRONT] Checkpoint Autônomo: Erro das rodas zerado!");
        }
        // if (mt2Left != null && mt2Left.tagCount > 0 && mt2Left.avgTagDist < 3.5 && velocidadeGiro < 45.0) {
        //     Pose2d currentPose = this.getState().Pose;
        //     Pose2d newPose = new Pose2d(mt2Left.pose.getTranslation(), currentPose.getRotation());
        //     this.resetPose(newPose);
        //     System.out.println("[VISÃO LEFT] Checkpoint Autônomo: Erro das rodas zerado!");
        // }
        // if (mt2Right != null && mt2Right.tagCount > 0 && mt2Right.avgTagDist < 3.5 && velocidadeGiro < 45.0) {
        //     Pose2d currentPose = this.getState().Pose;
        //     Pose2d newPose = new Pose2d(mt2Right.pose.getTranslation(), currentPose.getRotation());
        //     this.resetPose(newPose);
        //     System.out.println("[VISÃO RIGHT] Checkpoint Autônomo: Erro das rodas zerado!");
        // }
    }

    /**
    * Metodo utilizado para atualizar a odometria do robo de forma precisa quando o mesmo se encontra parado.
    * @param mt2 MegaTAG2 relacionado a camera alvo.
    */
    public void updateFrontCAM() {
        var mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue(frontCAM);

        double omega = Math.abs(this.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
        if (megaTagUpdateOdometry(mt2Front, 3.5, 45.0, omega)) {
            Pose2d currentPose = this.getState().Pose;
            Pose2d newPose = new Pose2d(mt2Front.pose.getTranslation(), currentPose.getRotation());

            this.resetPose(newPose);
            System.out.println("[VISÃO] Checkpoint Autônomo: Erro das rodas zerado!");
        }
    }

    public void configAngleInit() {
        double newAngle = isRedAlliance() ? 0.0 : 180.0;

        this.getPigeon2().setYaw(newAngle);
        try { Thread.sleep(20); } catch (Exception e) {}

        Pose2d currentPose = this.getState().Pose;
        this.resetPose(new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(newAngle)));
        
        System.out.println("Giroscópio Resetado fisicamente para: " + newAngle + " graus");
    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

}