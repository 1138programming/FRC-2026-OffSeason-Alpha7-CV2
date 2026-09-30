package first.robot.subsystems;

import java.util.concurrent.CancellationException;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.rotation.DutyCycleEncoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.AngularVelocity;


import static first.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase 
{
    private TalonFX IntakeRollerMotor1;
    private TalonFX IntakeRollerMotor2;
    private TalonFX IntakePivotMotor;

    private DutyCycleEncoder IntakePivotEncoder;

    private DutyCycleOut mintakePowerRequest;
    private NeutralOut mstopRequest;

    private PIDController intakePID;

    private boolean isDeployed;

    public Intake ()
    {
        IntakeRollerMotor1 = new TalonFX (kIntakeRoller1ID, kIntakeMotorCANBus);
        IntakeRollerMotor2 = new TalonFX (kIntakeRoller2ID, kIntakeMotorCANBus);
        IntakePivotMotor = new TalonFX (kIntakePivotID, kIntakeMotorCANBus);

        IntakePivotEncoder = new DutyCycleEncoder(kIntakePivotEncoderID, kIntakePivotZero, kIntakePivotDeployAngle);

        mintakePowerRequest = new DutyCycleOut(0);
        mstopRequest = new NeutralOut();

        intakePID = new PIDController(kIntakePIDp, kIntakePIDi, kIntakePIDd);

        isDeployed = false;
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

        IntakeRollerMotor1.getConfigurator().apply(roller1Config);
        IntakeRollerMotor2.getConfigurator().apply(roller2Config);
        IntakePivotMotor.getConfigurator().apply(pivotConfig);
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

    public double getIntakeEncoder()
    {
        return IntakePivotEncoder.get();
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

    public void intakePivotToPosition (double position)
    {
        double power = intakePID.calculate(getIntakeEncoder(), position);

        setIntakePivotPower(power);
    }

    public void resetIntakePIDPivot ()
    {
        intakePID.reset();
    }

    @Override
    public void periodic()
    {

    }
}
