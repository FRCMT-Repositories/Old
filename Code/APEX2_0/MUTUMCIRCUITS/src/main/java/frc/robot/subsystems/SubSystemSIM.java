package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class SubSystemSIM extends SubsystemBase {

    private double subGavetaPositon = 0.0;
    private double subIntakeAngle = 120;
    private double subIntakeVelocity = 0;
    private static double subShooterVelocity = 0;
    private double subClimberPositon = -0.1;

    private static double intakeAngleSim = 0.0;
    private static double climberPositionSim = -0.1;
    private double intakeVelocitySim = 0.0;
    private double shooterVelocitySim = 0.0;

    private PIDController intakePID = new PIDController(1.5, 0, 0.0);
    private PIDController climberPID = new PIDController(1.5, 0, 0.0);;
    private PIDController intakeVelocity = new PIDController(1.5, 0, 0.0);
    private PIDController shooterVelocity = new PIDController(10, 0, 0.0);

    // private final CommandSwerveDrivetrain swerve; CommandSwerveDrivetrain swerve

    public final class Intake {
        private final static double Real_Min = 0;
        private final static double Real_Max = 22.394;
        private final static double SIM_Min = 120;
        private final static double SIM_Max = 0;

        private final static double GavetaSIM_Min = 0.28;
        private final static double GavetaSIM_Max = 0;
    }

    public final class Climber {
        private final static double Real_Min = 0;
        private final static double Real_Max = 110;
        private final static double SIM_Min = -0.1;
        private final static double SIM_Max = 0.2;
    }

    public SubSystemSIM() {
        // this.swerve=swerve;
    }

    public void configIntake(double kP) {
        intakePID.setP(kP);
    }

    public void setIntakeVelocity(double speed) {
        subIntakeVelocity = speed;
    }

    public void setSubArticula(double angle, double kP) {
        configIntake(kP);
        subIntakeAngle = Hood.map(angle, Intake.Real_Min, Intake.Real_Max, Intake.SIM_Min, Intake.SIM_Max);
    }

    public static double getSubArticula() {
        return Hood.map(intakeAngleSim, Intake.SIM_Min, Intake.SIM_Max, Intake.Real_Min, Intake.Real_Max);
    }

    public void configClimber(double kP) {
        climberPID.setP(kP);
    }

    public double getIntakeVelocity() {
        return intakeVelocitySim;
    }

    public void intakeVelocityCurrent(double speedAtual) {
        intakeVelocitySim = speedAtual;
    }

    public double getShooterVelocity() {
        return shooterVelocitySim;
    }

    public static void setShooterVelocity(double speed) {
        subShooterVelocity = speed;
    }

    public void shooterVelocityCurrent(double speedAtual) {
        shooterVelocitySim = speedAtual;
    }

    public void setSubClimber(Pose2d robotPose, double position, double kP) {
        configClimber(kP);

        double blueX = 4.298; // Aliança
        double redX = 12.41; // Aliança

        if ((robotPose.getX() >= blueX - 0.2 && robotPose.getX() <= (blueX - 0.2) + 1.2)
            || (robotPose.getX() >= (redX + 0.2) - 1.2 && robotPose.getX() <= redX + 0.2)) {
            if (position < 10) {
                subClimberPositon = Hood.map(position, Climber.Real_Min, Climber.Real_Max, Climber.SIM_Min, Climber.SIM_Max);
            }
        }
        else {
            subClimberPositon = Hood.map(position, Climber.Real_Min, Climber.Real_Max, Climber.SIM_Min, Climber.SIM_Max);
        }
    }

    public static double getSubClimber() {
        return Hood.map(climberPositionSim, Climber.SIM_Min, Climber.SIM_Max, Climber.Real_Min, Climber.Real_Max);
    }

    // simulationPeriodic
    @Override
    public void periodic() {

        double intakeOutput = intakePID.calculate(intakeAngleSim, subIntakeAngle);
        intakeAngleSim += intakeOutput * 0.02;
        intakeAngleSim = MathUtil.clamp(intakeAngleSim, Intake.SIM_Max, Intake.SIM_Min);

        subGavetaPositon = Hood.map(intakeAngleSim, Intake.SIM_Max, Intake.SIM_Min, Intake.GavetaSIM_Min, Intake.GavetaSIM_Max);

        double climberOutput = climberPID.calculate(climberPositionSim, subClimberPositon);
        climberPositionSim += climberOutput * 0.02;

        double intakeVelocityOutput = intakeVelocity.calculate(intakeVelocitySim, subIntakeVelocity);
        intakeVelocitySim += intakeVelocityOutput * 0.02;

        double shooterVelocityOutput = shooterVelocity.calculate(shooterVelocitySim, subShooterVelocity);
        shooterVelocitySim += shooterVelocityOutput * 0.02;

        Logger.recordOutput("SubSystemSim/IntakeVelocity", intakeVelocitySim);
        Logger.recordOutput("SubSystemSim/ShooterVelocity", shooterVelocitySim);
        Logger.recordOutput("SubSystemSim/Position/Articula", getSubArticula());
        Logger.recordOutput("SubSystemSim/Position/Climber", getSubClimber());

        Logger.recordOutput("SubSystemSim/Hood3D", new Pose3d[] { new Pose3d(
                0.345, 0, 0.43, new Rotation3d(0.0, Math.toRadians(Hood.getAngleHoodSim()), Math.PI)) });

        Logger.recordOutput("SubSystemSim/Intake3D", new Pose3d[] { new Pose3d(
                -0.24, 0, 0.178, new Rotation3d(0.0, Math.toRadians(intakeAngleSim), 0)) });

        Logger.recordOutput("SubSystemSim/Gaveta3D", new Pose3d[] { new Pose3d(
                0.0552 - subGavetaPositon, 0, 0.179, new Rotation3d(0.0, 0, 0)) });

        Logger.recordOutput("SubSystemSim/Climber3D", new Pose3d[] { new Pose3d(
                0.356, 0, climberPositionSim, new Rotation3d(0.0, 0, 0)) });
    }
}
