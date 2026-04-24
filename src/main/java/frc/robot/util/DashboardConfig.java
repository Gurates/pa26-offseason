package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.ArmSubsystem;

public class DashboardConfig {

    private final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain drivetrain;
    private final ArmSubsystem arm;

    public DashboardConfig(CommandSwerveDrivetrain drivetrain, ArmSubsystem arm) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        SmartDashboard.putData("Field", field);
        SmartDashboard.putBoolean("Hub Active", false);

        SmartDashboard.putNumber("Intake Angle", 0.0);

        setupUsbCamera();

    }

    private void setupUsbCamera() {
        try {
            CameraServer.startAutomaticCapture("Intake Camera", 0);
        } catch (Exception e) {
            System.err.println("[Dashboard] USB kamera ayarlanamadı: " + e.getMessage());
        }
    }

    public void update() {
        Pose2d pose = drivetrain.getState().Pose;
        if (pose != null) {
            field.setRobotPose(pose);
        }

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        SmartDashboard.putBoolean("Hub Active", isHubActive());

        SmartDashboard.putNumber("Intake Angle", getIntakeAngleDegrees());
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            return true;              // Geçiş süresi
        } else if (matchTime > 105) {
            return shift1Active;      // Shift 1
        } else if (matchTime > 80) {
            return !shift1Active;     // Shift 2
        } else if (matchTime > 55) {
            return shift1Active;      // Shift 3
        } else if (matchTime > 30) {
            return !shift1Active;     // Shift 4
        } else {
            return true;              // Endgame
        }
    }

    private double getIntakeAngleDegrees() {
        return arm.getAngleDeg();
    }
}