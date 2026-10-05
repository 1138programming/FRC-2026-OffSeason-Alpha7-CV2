package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.rotation.DutyCycleEncoder;
import org.wpilib.math.util.MathUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;


import static first.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase 
{
    private TalonFX IntakeRollerMotor1;
    private TalonFX IntakeRollerMotor2;
    private TalonFX IntakePivotMotor;

    private DutyCycleEncoder IntakePivotEncoder;

    private DutyCycleOut mintakePowerRequest;
    private NeutralOut mstopRequest;

    private final MotionMagicTorqueCurrentFOC mintakePositionRequest =
        new MotionMagicTorqueCurrentFOC(0).withSlot(0).withUpdateFreqHz(0);
    private boolean pivotConfigured;
    private boolean pivotPositionInitialized;
    private final Alert pivotControlUnavailableAlert = new Alert(
        "intakePivotControlUnavailable",
        "Intake pivot position control unavailable. Check motor configuration and absolute encoder; disable to initialize.",
        Level.HIGH);

    private boolean isDeployed;

    public Intake ()
    {
        IntakeRollerMotor1 = new TalonFX (kIntakeRoller1ID, kIntakeMotorCANBus);
        IntakeRollerMotor2 = new TalonFX (kIntakeRoller2ID, kIntakeMotorCANBus);
        IntakePivotMotor = new TalonFX (kIntakePivotID, kIntakeMotorCANBus);

        IntakePivotEncoder = new DutyCycleEncoder(
            kIntakePivotEncoderID, 360.0, kIntakePivotEncoderZeroDegrees);
        IntakePivotEncoder.setInverted(kIntakePivotEncoderInverted);

        mintakePowerRequest = new DutyCycleOut(0);
        mstopRequest = new NeutralOut();

        isDeployed = false;
        configureIntakeMotors();
    }

    public void configureIntakeMotors()
    {
        final TalonFXConfiguration roller1Config = new TalonFXConfiguration()
            .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            );

        final TalonFXConfiguration roller2Config = new TalonFXConfiguration()
            .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            );

        final TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
            .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            );

        // The output-shaft absolute encoder initializes the rotor position while disabled.
        // The Kraken then runs the profile and position loop using its internal rotor sensor.
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfig.Feedback.SensorToMechanismRatio = kIntakePivotGearRatio;
        pivotConfig.Slot0.kP = kIntakePIDp;
        pivotConfig.Slot0.kI = kIntakePIDi;
        pivotConfig.Slot0.kD = kIntakePIDd;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = kIntakePivotCruiseVelocity / 360.0;
        pivotConfig.MotionMagic.MotionMagicAcceleration = kIntakePivotAcceleration / 360.0;
        pivotConfig.MotionMagic.MotionMagicJerk = kIntakePivotJerk / 360.0;
        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = kIntakePivotCurrentLimitAmps;
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kIntakePivotCurrentLimitAmps;
        pivotConfig.CurrentLimits.StatorCurrentLimit = kIntakePivotCurrentLimitAmps;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kIntakePivotZero / 360.0;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kIntakePivotDeployAngle / 360.0;

        stopIntakePivot();
        pivotPositionInitialized = false;
        IntakeRollerMotor1.getConfigurator().apply(roller1Config);
        IntakeRollerMotor2.getConfigurator().apply(roller2Config);
        pivotConfigured = IntakePivotMotor.getConfigurator().apply(pivotConfig).isOK();
        pivotControlUnavailableAlert.set(true);
    }

    public void stopIntakeRollers ()
    {
        IntakeRollerMotor1.setControl(mstopRequest);
        IntakeRollerMotor2.setControl(mstopRequest);
    }

    public void stopIntakePivot ()
    {
        IntakePivotMotor.setControl(mstopRequest);
    }

    public void setIntakeRollerPower(double power)
    {
        IntakeRollerMotor1.setControl(mintakePowerRequest.withOutput(power));
        IntakeRollerMotor2.setControl(mintakePowerRequest.withOutput(power));
    }

    public void setIntakePivotPower(double power)
    {
        IntakePivotMotor.setControl(mintakePowerRequest.withOutput(power));
    }

    public void setDeployed (boolean deployed)
    {
        isDeployed = deployed;
    }

    /** Returns output-shaft angle in degrees relative to the calibrated stow position. */
    public double getIntakeEncoder()
    {
        // Avoid interpreting an angle just below stow as almost a full revolution.
        return MathUtil.inputModulus(IntakePivotEncoder.get(), -180.0, 180.0);
    }

    public double getIntakeRollerVelocityRPM()
    {
        StatusSignal<AngularVelocity> v1 = IntakeRollerMotor1.getVelocity();
        StatusSignal<AngularVelocity> v2 = IntakeRollerMotor2.getVelocity();

        v1.refresh();
        v2.refresh();

        return ((v1.getValueAsDouble() + v2.getValueAsDouble()) / 2) * 60;
    }

    public boolean isDeployed ()
    {
        return isDeployed;
    }

    /** Commands an output-shaft position in degrees with onboard Motion Magic. */
    public void intakePivotToPosition (double position)
    {
        if (!pivotConfigured || !pivotPositionInitialized || !Double.isFinite(position)) {
            stopIntakePivot();
            return;
        }
        double targetDegrees = Math.clamp(position, kIntakePivotZero, kIntakePivotDeployAngle);
        IntakePivotMotor.setControl(mintakePositionRequest.withPosition(targetDegrees / 360.0));
    }

    public void resetIntakePIDPivot ()
    {
        // Retain the existing API; resynchronize the onboard position on the next disabled loop.
        stopIntakePivot();
        pivotPositionInitialized = false;
    }

    @Override
    public void periodic()
    {
        if (IntakePivotMotor.hasResetOccurred()) {
            pivotPositionInitialized = false;
            stopIntakePivot();
        }
        if (RobotState.isDisabled() && pivotConfigured && !pivotPositionInitialized
            && IntakePivotEncoder.isConnected()) {
            double angleDegrees = getIntakeEncoder();
            if (Double.isFinite(angleDegrees)) {
                pivotPositionInitialized = IntakePivotMotor.setPosition(angleDegrees / 360.0).isOK();
            }
        }
        pivotControlUnavailableAlert.set(!pivotConfigured || !pivotPositionInitialized);
    }
}
