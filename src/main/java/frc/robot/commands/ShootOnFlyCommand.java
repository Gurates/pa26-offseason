package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

public class ShootOnFlyCommand extends Command {

    private static final Translation2d BLUE_HUB = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB  = new Translation2d(11.961, 4.021);

    private static final double KP_ROTATION           = 0.065;
    private static final double MIN_COMMAND_RADPS     = 0.03;
    private static final double ERROR_DEADZONE_DEG    = 1.5;
    private static final double ALIGNED_TOLERANCE_DEG = 2.5;
    private static final double MAX_ROT_RADPS         = Units.rotationsToRadians(2.0);
    private static final double JOYSTICK_DEADBAND     = 0.10;

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier leftXSupplier;
    private final double maxSpeed;

    private boolean isAligned        = false;
    private boolean isRightDirection = false;

    private final NetworkTable     nt;
    private final DoublePublisher  ntRobotX;
    private final DoublePublisher  ntRobotY;
    private final DoublePublisher  ntRobotHeadingDeg;
    private final DoublePublisher  ntTargetHeadingDeg;
    private final DoublePublisher  ntRotErrorDeg;
    private final DoublePublisher  ntRotOutputRadps;
    private final DoublePublisher  ntDistanceToHubM;
    private final BooleanPublisher ntAligned;
    private final BooleanPublisher ntRunning;
    private final StringPublisher  ntAlliance;

    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ShootOnFlyCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier,
            double maxSpeed) {

        this.drivetrain    = drivetrain;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.maxSpeed      = maxSpeed;

        addRequirements(drivetrain);

        nt = NetworkTableInstance.getDefault().getTable("ContinuousAim");
        ntRobotX           = nt.getDoubleTopic("Robot_X_m").publish();
        ntRobotY           = nt.getDoubleTopic("Robot_Y_m").publish();
        ntRobotHeadingDeg  = nt.getDoubleTopic("Robot_Heading_deg").publish();
        ntTargetHeadingDeg = nt.getDoubleTopic("Target_Heading_deg").publish();
        ntRotErrorDeg      = nt.getDoubleTopic("Rotation_Error_deg").publish();
        ntRotOutputRadps   = nt.getDoubleTopic("Rotation_Output_radps").publish();
        ntDistanceToHubM   = nt.getDoubleTopic("Distance_To_Hub_m").publish();
        ntAligned          = nt.getBooleanTopic("Aligned").publish();
        ntRunning          = nt.getBooleanTopic("Running").publish();
        ntAlliance         = nt.getStringTopic("Alliance").publish();
    }

    @Override
    public void initialize() {
        isAligned = false;
        ntRunning.set(true);
        ntAligned.set(false);
    }

    @Override
    public void execute() {
        // Calculate Speed
        double vx = drivetrain.getState().Speeds.vxMetersPerSecond;
        double vy = drivetrain.getState().Speeds.vyMetersPerSecond;
        double totalSpeed = Math.hypot(vx, vy);

        double forwardMps = applyDeadband(leftYSupplier.getAsDouble()) * -maxSpeed;
        double strafeMps  = applyDeadband(leftXSupplier.getAsDouble()) * -maxSpeed;

        Pose2d robotPose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;

        Translation2d toHub = hub.minus(robotPose.getTranslation());
        double distance = toHub.getNorm();
        Rotation2d targetHdg = new Rotation2d(toHub.getX(), toHub.getY());

        Rotation2d dgOffset = calculateAngle(Math.abs(vy), 0.75);

        if (vy > 0.1) {
            isRightDirection = false;
        } else if (vy < -0.1) {
            isRightDirection = true;
        }

        if (!isRightDirection) {
            targetHdg = targetHdg.minus(dgOffset);
        } else {
            targetHdg = targetHdg.plus(dgOffset);
        }
        double errorDeg = targetHdg.minus(robotPose.getRotation()).getDegrees();

        // P-controller
        double output;
        if (Math.abs(errorDeg) > ERROR_DEADZONE_DEG) {
            output  = KP_ROTATION * errorDeg;
            output += (errorDeg > 0) ? MIN_COMMAND_RADPS : -MIN_COMMAND_RADPS;
        } else {
            output = KP_ROTATION * errorDeg;
        }
        output = Math.max(-MAX_ROT_RADPS, Math.min(MAX_ROT_RADPS, output));

        isAligned = Math.abs(errorDeg) <= ALIGNED_TOLERANCE_DEG;

        drivetrain.setControl(
                driveRequest
                        .withVelocityX(forwardMps)
                        .withVelocityY(strafeMps)
                        .withRotationalRate(output));

        // Telemetri
        ntRobotX.set(robotPose.getX());
        ntRobotY.set(robotPose.getY());
        ntRobotHeadingDeg.set(robotPose.getRotation().getDegrees());
        ntTargetHeadingDeg.set(targetHdg.getDegrees());
        ntRotErrorDeg.set(errorDeg);
        ntRotOutputRadps.set(output);
        ntDistanceToHubM.set(distance);
        ntAligned.set(isAligned);
        ntAlliance.set(alliance.toString());
    }

    @Override
    public void end(boolean interrupted) {
        ntRunning.set(false);
        ntAligned.set(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isAligned() {
        return isAligned;
    }

    public double getDistanceToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;
        return hub.minus(robotPose.getTranslation()).getNorm();
    }

    private static double applyDeadband(double value) {
        return Math.abs(value) > JOYSTICK_DEADBAND ? value : 0.0;
    }

    public static Rotation2d calculateAngle(double vRun, double vForward) {
        return new Rotation2d(Math.atan2(vRun, vForward));
    }
}