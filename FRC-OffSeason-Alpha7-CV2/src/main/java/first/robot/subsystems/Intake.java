package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.rotation.DutyCycleEncoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.AngularVelocity;


import static first.robot.Constants.IntakeConstants.*;
import static first.robot.subsystems.drive.ModuleIOTalonFX.tryUntilOk;

public class Intake extends SubsystemBase
{
    private TalonFX IntakeRollerMotor1;
    private TalonFX IntakeRollerMotor2;
    private TalonFX IntakePivotMotor;

    private DutyCycleEncoder IntakePivotEncoder;

    private DutyCycleOut mintakePowerRequest;
    private PositionVoltage mpivotPositionRequest;
    private NeutralOut mstopRequest;

    private boolean isDeployed;
    private boolean isPivotSeeded;

    public Intake ()
    {
        IntakeRollerMotor1 = new TalonFX (kINTAKE_ROLLER_1_ID, new CANBus(kIntakeMotorCANPort));
        IntakeRollerMotor2 = new TalonFX (kINTAKE_ROLLER_2_ID,  new CANBus(kIntakeMotorCANPort));
        IntakePivotMotor = new TalonFX (kINTAKE_PIVOT_ID, new CANBus(kIntakeMotorCANPort));

        IntakeRollerMotor2.setControl(new Follower(kINTAKE_ROLLER_1_ID, kINTAKE_ROLLER_2_ALIGNMENT));

        configureIntakeMotors();

        IntakePivotEncoder = new DutyCycleEncoder(kIntakePivotEncoderID, 360.0, kIntakePivotEncoderOffset);
        IntakePivotEncoder.setInverted(kIntakePivotEncoderInverted);

        configureIntakeMotors();

        
        IntakeRollerMotor2.setControl(new Follower(kINTAKE_ROLLER_1_ID, kINTAKE_ROLLER_2_ALIGNMENT));

        mintakePowerRequest = new DutyCycleOut(0);
        mpivotPositionRequest = new PositionVoltage(0).withSlot(0);
        mstopRequest = new NeutralOut();

        isDeployed = false;
        isPivotSeeded = false;
    }

    private void configureIntakeMotors()
    {

        final TalonFXConfiguration rollerConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            )
            .withSlot0(
                new Slot0Configs()
                .withKP(kINTAKE_P)
                .withKI(kINTAKE_I)
                .withKD(kINTAKE_D)
            );

        final TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
            .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            )
            .withFeedback(
            new FeedbackConfigs()
            .withSensorToMechanismRatio(kINTAKE_PIVOT_GEAR_RATIO)
            )
            .withSlot0(
            new Slot0Configs()
            .withKP(kINTAKE_P)
            .withKI(kINTAKE_I)
            .withKD(kINTAKE_D)
            );

        tryUntilOk(5, () -> IntakeRollerMotor1.getConfigurator().apply(rollerConfig, 0.25));
        tryUntilOk(5, () -> IntakeRollerMotor2.getConfigurator().apply(rollerConfig, 0.25));
        tryUntilOk(5, () -> IntakePivotMotor.getConfigurator().apply(pivotConfig, 0.25));
    }

    public void stopIntakeRollers ()
    {
        IntakeRollerMotor1.setControl(mstopRequest);
    }

    public void stopIntakePivot ()
    {
        IntakePivotMotor.setControl(mstopRequest);
    }

    public void setIntakeRollerPower(double power)
    {
        IntakeRollerMotor1.setControl(mintakePowerRequest.withOutput(power));
    }

    public void setIntakePivotPower(double power)
    {
        IntakePivotMotor.setControl(mintakePowerRequest.withOutput(power));
    }

    public void setDeployed (boolean deployed)
    {
        isDeployed = deployed;
    }

    /** Pivot angle in degrees from the Through Bore, wrapped to (-180, 180] so slightly past stow reads negative. */
    public double getIntakeEncoder()
    {
        double angle = IntakePivotEncoder.get();
        return angle > 180.0 ? angle - 360.0 : angle;
    }

    /** True when the pivot is within tolerance of the target angle in degrees. */
    public boolean isPivotAtPosition(double position)
    {
        return Math.abs(getIntakeEncoder() - position) <= kIntakePivotToleranceDegrees;
    }

    /** Copies the Through Bore angle into the TalonFX so its onboard PID uses the real pivot position. */
    public void seedPivotFromEncoder()
    {
        if (IntakePivotEncoder.isConnected())
        {
            IntakePivotMotor.setPosition(degreesToRotations(getIntakeEncoder()));
            isPivotSeeded = true;
        }
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

    /** Runs the TalonFX onboard position PID to the given pivot angle in degrees. */
    public void intakePivotToPosition (double position)
    {
        if (!isPivotSeeded)
        {
            stopIntakePivot();
            return;
        }

        IntakePivotMotor.setControl(mpivotPositionRequest.withPosition(degreesToRotations(position)));
    }

    private static double degreesToRotations(double degrees)
    {
        return degrees / 360.0;
    }

    @Override
    public void periodic()
    {
        if (!isPivotSeeded)
        {
            seedPivotFromEncoder();
        }
    }
}
