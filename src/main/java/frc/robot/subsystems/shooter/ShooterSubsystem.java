package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX motor;

    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

    public double targetWheelRPM = 0.0;
    private boolean motorConfigured = false;

    private double spinupStartTime = -1.0;
    private boolean isSpinningUp = false;
    public double currentRPM = 0;

    public ShooterSubsystem() {
        motor = new TalonFX(ShooterConstants.MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = ShooterConstants.kP_TALON;
        slot0.kI = ShooterConstants.kI_TALON;
        slot0.kD = ShooterConstants.kD_TALON;
        slot0.kV = ShooterConstants.kV_TALON;
        slot0.kS = ShooterConstants.kS_TALON;

        motor.getConfigurator().apply(config);
        motorConfigured = true;

        SmartDashboard.putNumber("Shooter/Spinup Wait (s)", ShooterConstants.SPINUP_WAIT_SECONDS);
    }

    @Override
    public void periodic() {
        double waitTime = SmartDashboard.getNumber("Shooter/Spinup Wait (s)", ShooterConstants.SPINUP_WAIT_SECONDS);

        if (isSpinningUp) {
            double elapsed = Timer.getFPGATimestamp() - spinupStartTime;
            if (elapsed >= waitTime && atTargetVelocity()) {
                isSpinningUp = false;
            }
        }

        double elapsed = isSpinningUp ? (Timer.getFPGATimestamp() - spinupStartTime) : 0.0;

        SmartDashboard.putNumber("Shooter/Target Wheel RPM", targetWheelRPM);
        SmartDashboard.putNumber("Shooter/Target Motor RPM", targetWheelRPM * ShooterConstants.GEAR_RATIO);
        SmartDashboard.putNumber("Shooter/Actual Wheel RPM", getWheelRPM());
        SmartDashboard.putNumber("Shooter/Actual Motor RPM", getMotorRPM());
        SmartDashboard.putNumber("Shooter/RPM Error", targetWheelRPM - getWheelRPM());
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        SmartDashboard.putBoolean("Shooter/Is Spinning Up", isSpinningUp);
        SmartDashboard.putNumber("Shooter/Spinup Elapsed (s)", elapsed);
        SmartDashboard.putNumber("Shooter/Spinup Remaining (s)",
            isSpinningUp ? Math.max(0, waitTime - elapsed) : 0.0);
        SmartDashboard.putBoolean("Shooter/Motor Configured", motorConfigured);

        SmartDashboard.putNumber("Shooter/Config Angle (deg)", ShooterConstants.SHOOTER_ANGLE_DEGREES);
        SmartDashboard.putNumber("Shooter/Config Min Dist (m)", ShooterConstants.MIN_SAFE_DISTANCE);
        SmartDashboard.putNumber("Shooter/Config Max Dist (m)", ShooterConstants.MAX_SAFE_DISTANCE);
    }

    public void setVelocityForDistance(double distanceMeters) {
        double wheelRPM = getRPMForDistance(distanceMeters);
        setVelocityRPM(wheelRPM);
        SmartDashboard.putNumber("Shooter/Last Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Last Commanded RPM", wheelRPM);
    }

    public void updateVelocityForDistance(double distanceMeters) {
        double wheelRPM = getRPMForDistance(distanceMeters);

        if (Math.abs(wheelRPM - targetWheelRPM) < 50.0) return;

        targetWheelRPM = wheelRPM;
        double motorRPS = (wheelRPM * ShooterConstants.GEAR_RATIO) / 60.0;
        motor.setControl(velocityRequest.withVelocity(motorRPS));
        SmartDashboard.putNumber("Shooter/Last Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Last Commanded RPM", wheelRPM);
    }

    public void setVelocityRPM(double wheelRPM) {
        targetWheelRPM = wheelRPM;
        double motorRPS = (wheelRPM * ShooterConstants.GEAR_RATIO) / 60.0;
        motor.setControl(velocityRequest.withVelocity(motorRPS));
        spinupStartTime = Timer.getFPGATimestamp();
        isSpinningUp = true;
    }

    public void stop() {
        motor.stopMotor();
        targetWheelRPM = 0.0;
        isSpinningUp = false;
        spinupStartTime = -1.0;
    }

    public void changeVelocityRPM(double deltaRPM) {
        double newTarget = targetWheelRPM + deltaRPM;
        if (newTarget < 0) newTarget = 0;

        targetWheelRPM = newTarget;
        double motorRPS = (targetWheelRPM * ShooterConstants.GEAR_RATIO) / 60.0;
        
        motor.setControl(velocityRequest.withVelocity(motorRPS));

        if (Math.abs(deltaRPM) > 500.0) {
            spinupStartTime = Timer.getFPGATimestamp();
            isSpinningUp = true;
        }
    }

    public double getWheelRPM() {
        return getMotorRPM() / ShooterConstants.GEAR_RATIO;
    }

    public double getMotorRPM() {
        return motor.getVelocity().getValueAsDouble() * 60.0;
    }

    public boolean atTargetVelocity() {
        if (targetWheelRPM == 0.0) return false;
        return Math.abs(getWheelRPM() - targetWheelRPM) < ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }

    public boolean isReadyToShoot() {
        return atTargetVelocity() && !isSpinningUp;
    }

    public boolean isDistanceInRange(double distanceMeters) {
        return distanceMeters >= ShooterConstants.MIN_SAFE_DISTANCE && 
               distanceMeters <= ShooterConstants.MAX_SAFE_DISTANCE;
    }

    private double getRPMForDistance(double distance) {
        if (distance < ShooterConstants.MIN_SAFE_DISTANCE) {
            DriverStation.reportWarning(String.format(
                "Distance %.2fm below min %.2fm - clamping", distance, ShooterConstants.MIN_SAFE_DISTANCE), false);
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return ShooterConstants.DISTANCE_TO_RPM_MAP.get(ShooterConstants.MIN_SAFE_DISTANCE);
        }
        if (distance > ShooterConstants.MAX_SAFE_DISTANCE) {
            DriverStation.reportWarning(String.format(
                "Distance %.2fm exceeds max %.2fm - clamping", distance, ShooterConstants.MAX_SAFE_DISTANCE), false);
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return ShooterConstants.DISTANCE_TO_RPM_MAP.get(ShooterConstants.MAX_SAFE_DISTANCE);
        }
        SmartDashboard.putBoolean("Shooter/Distance In Range", true);
        return ShooterConstants.DISTANCE_TO_RPM_MAP.get(distance);
    }
}