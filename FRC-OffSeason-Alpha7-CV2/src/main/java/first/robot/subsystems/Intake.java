package first.robot.subsystems;

import java.util.concurrent.CancellationException;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.rotation.DutyCycleEncoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
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

public class Intake extends SubsystemBase 
{
    private TalonFX IntakeRollerMotor1;
    private TalonFX IntakeRollerMotor2;
    private TalonFX IntakePivotMotor;

    private DutyCycleEncoder IntakePivotEncoder;

    private DutyCycleOut mintakePowerRequest;
    private PositionVoltage mintakePositionRequest;
    private NeutralOut mstopRequest;

    private PIDController intakePID;

    private boolean isDeployed;

    public Intake ()
    {
        IntakeRollerMotor1 = new TalonFX (kINTAKE_ROLLER_1_ID, new CANBus(kINTAKE_MOTOR_CAN_PORT));
        IntakeRollerMotor2 = new TalonFX (kINTAKE_ROLLER_2_ID,  new CANBus(kINTAKE_MOTOR_CAN_PORT));
        IntakePivotMotor = new TalonFX (kINTAKE_PIVOT_ID, new CANBus(kINTAKE_MOTOR_CAN_PORT));

        IntakeRollerMotor2.setControl(new Follower(kINTAKE_ROLLER_1_ID, kINTAKE_ROLLER_2_ALIGNMENT));

        configureIntakeMotors();

        IntakePivotEncoder = new DutyCycleEncoder(kINTAKE_PIVOT_ENCODER_ID, 360, kINTAKE_PIVOT_ZERO);

        mintakePowerRequest = new DutyCycleOut(0);
        mintakePositionRequest = new PositionVoltage(0).withSlot(0);
        mstopRequest = new NeutralOut();

        intakePID = new PIDController(kINTAKE_P, kINTAKE_I, kINTAKE_D);

        isDeployed = false;
    }

    public void configureIntakeMotors()
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
            );

        IntakeRollerMotor1.getConfigurator().apply(rollerConfig);
        IntakeRollerMotor2.getConfigurator().apply(rollerConfig);
        IntakePivotMotor.getConfigurator().apply(pivotConfig);
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
