package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;

    private static final int ROLLER_MOTOR_ID = 37;
    private static final int CURRENT_LIMIT = 40;
    private static final double GEAR_RATIO = 1.0;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    private double currentOutput = 0.0;

    public IntakeRollerSubsystem() {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = 0.1;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.12;
        slot0.kS = 0.05;

        rollerMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeRoller/Output",currentOutput);
        SmartDashboard.putNumber("IntakeRoller/Actual RPM",getActualRPM());
        SmartDashboard.putNumber("IntakeRoller/Stator (A)",rollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("IntakeRoller/Supply (A)",rollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("IntakeRoller/Running",isRunning());
    }

    public void intake() {
        setRPM(3000);
    }

    public void eject() {
        setRPM(-3000);
    }

    public void stop() {
        currentOutput = 0.0;
        rollerMotor.stopMotor();
    }

    public void setRPM(double wheelRPM) {
        currentOutput = wheelRPM;
        double motorRPS = (wheelRPM * GEAR_RATIO) / 60.0;
        rollerMotor.setControl(velocityRequest.withVelocity(motorRPS));
    }

    public void setOutput(double output) {
        currentOutput = output;
        rollerMotor.setControl(dutyCycleRequest.withOutput(output));
    }

    public double getActualRPM() {
        return (rollerMotor.getVelocity().getValueAsDouble() * 60.0) / GEAR_RATIO;
    }

    public boolean isRunning() {
        return Math.abs(currentOutput) > 0.01;
    }

    public double getStatorCurrent() {
        return rollerMotor.getStatorCurrent().getValueAsDouble();
    }
}