package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.constants.HoodConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodSubsystem extends SubsystemBase {

    private final SparkMax hoodMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double targetPosition = HoodConstants.HOOD_OPENED_DEGREES;
    private boolean manualMode = false;
    private boolean holdingPosition = false;

    public HoodSubsystem() {
        hoodMotor = new SparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(30)
              .inverted(false);

        double positionConversionFactor = 360.0 / HoodConstants.HOOD_GEAR_RATIO;
        config.encoder
              .positionConversionFactor(positionConversionFactor)
              .velocityConversionFactor(positionConversionFactor / 60.0);

        config.closedLoop
              .pid(HoodConstants.HOOD_kP, HoodConstants.HOOD_kI, HoodConstants.HOOD_kD)
              .positionWrappingEnabled(false);

        config.softLimit
              .forwardSoftLimit(HoodConstants.HOOD_CLOSED_DEGREES - 5)
              .forwardSoftLimitEnabled(true)
              .reverseSoftLimit(HoodConstants.HOOD_OPENED_DEGREES + 5)
              .reverseSoftLimitEnabled(true);

        config.signals
              .outputCurrentPeriodMs(20)
              .appliedOutputPeriodMs(20)
              .busVoltagePeriodMs(20);

        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodMotor.clearFaults();

        encoder = hoodMotor.getEncoder();
        pidController = hoodMotor.getClosedLoopController();

        encoder.setPosition(HoodConstants.HOOD_OPENED_DEGREES);
    }

    public void close() {
        setTargetPosition(HoodConstants.HOOD_CLOSED_DEGREES);
    }

    public void open() {
        setTargetPosition(HoodConstants.HOOD_OPENED_DEGREES);
    }
    public void toggle() {
        if (isOpen()) {
            close();
        } else {
            open();
        }
    }

    public void setAngle(double degrees) {
        double clamped = MathUtil.clamp(
            degrees,
            HoodConstants.HOOD_OPENED_DEGREES,
            HoodConstants.HOOD_CLOSED_DEGREES
        );
        setTargetPosition(clamped);
    }

    public void setAngleForDistance(double distanceMeters) {
        double angle = HoodConstants.DISTANCE_TO_HOOD_ANGLE_MAP.get(
            MathUtil.clamp(
                distanceMeters,
                HoodConstants.MIN_SAFE_DISTANCE,
                HoodConstants.MAX_SAFE_DISTANCE
            )
        );
        setTargetPosition(angle);
        SmartDashboard.putNumber("Hood/Target Angle For Distance", angle);
    }

    public void setSpeed(double speed) {
        manualMode = true;
        holdingPosition = false;
        hoodMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
        targetPosition = encoder.getPosition();
    }

    public void stop() {
        targetPosition = encoder.getPosition();
        manualMode = false;
        holdingPosition = false;
        pidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void calibrate() {
        hoodMotor.stopMotor();
        encoder.setPosition(HoodConstants.HOOD_OPENED_DEGREES);
        targetPosition  = HoodConstants.HOOD_OPENED_DEGREES;
        manualMode      = false;
        holdingPosition = true;
        System.out.println("[Hood] Encoder calibrated to " + HoodConstants.HOOD_OPENED_DEGREES + " degrees");
    }

    public boolean isOpen() {
        return Math.abs(encoder.getPosition() - HoodConstants.HOOD_OPENED_DEGREES)
               < HoodConstants.HOOD_POSITION_TOLERANCE;
    }

    public boolean isClosed() {
        return Math.abs(encoder.getPosition() - HoodConstants.HOOD_CLOSED_DEGREES)
               < HoodConstants.HOOD_POSITION_TOLERANCE;
    }

    public boolean atTarget() {
        boolean positionOk = Math.abs(encoder.getPosition() - targetPosition)
                             < HoodConstants.HOOD_POSITION_TOLERANCE;
        boolean velocityOk = Math.abs(getVelocity()) < HoodConstants.HOOD_SETTLED_VELOCITY_THRESHOLD;
        return positionOk && velocityOk;
    }

    public double getPosition()       { return encoder.getPosition(); }
    public double getVelocity()       { return encoder.getVelocity(); }
    public double getTargetPosition() { return targetPosition; }
    public double getMotorCurrent()   { return hoodMotor.getOutputCurrent(); }
    public boolean isManualMode()     { return manualMode; }

    private void setTargetPosition(double positionDeg) {
        holdingPosition = false;
        manualMode      = false;
        targetPosition  = positionDeg;
        pidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }


    @Override
    public void periodic() {
        boolean isStalled = !manualMode && !holdingPosition
                && Math.abs(getVelocity()) < HoodConstants.HOOD_SETTLED_VELOCITY_THRESHOLD
                && Math.abs(targetPosition - getPosition()) > HoodConstants.HOOD_POSITION_TOLERANCE;

        SmartDashboard.putNumber("Hood/Position (deg)",        getPosition());
        SmartDashboard.putNumber("Hood/Target Position (deg)", targetPosition);
        SmartDashboard.putNumber("Hood/Velocity (deg/s)",      getVelocity());
        SmartDashboard.putNumber("Hood/Position Error",        targetPosition - getPosition());
        SmartDashboard.putBoolean("Hood/Is Open",              isOpen());
        SmartDashboard.putBoolean("Hood/Is Closed",            isClosed());
        SmartDashboard.putBoolean("Hood/At Target",            atTarget());
        SmartDashboard.putBoolean("Hood/Holding Position",     holdingPosition);
        SmartDashboard.putBoolean("Hood/Manual Mode",          manualMode);
        SmartDashboard.putBoolean("Hood/Stalled",              isStalled);
        SmartDashboard.putNumber("Hood/Motor Current (A)",     getMotorCurrent());
        SmartDashboard.putNumber("Hood/Applied Output",        hoodMotor.getAppliedOutput());

        boolean outOfBounds = getPosition() < (HoodConstants.HOOD_OPENED_DEGREES - 15)
                           || getPosition() > (HoodConstants.HOOD_CLOSED_DEGREES + 15);
        SmartDashboard.putBoolean("Hood/Out of Bounds", outOfBounds);
    }
}