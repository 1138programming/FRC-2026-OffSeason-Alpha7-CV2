package first.robot.subsystems;

import java.util.concurrent.CancellationException;

import org.wpilib.command2.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import first.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase 
{
    TalonFX IntakeRollerMotor1;
    TalonFX IntakeRollerMotor2;
    TalonFX IntakePivotMotor;

    private DutyCycleOut powerRequest;

    public Intake ()
    {
        IntakeRollerMotor1 = new TalonFX (IntakeConstants.kIntakeRoller1ID, IntakeConstants.kIntakeMotorCANBus);
        IntakeRollerMotor2 = new TalonFX (IntakeConstants.kIntakeRoller2ID, IntakeConstants.kIntakeMotorCANBus);
        IntakePivotMotor = new TalonFX (IntakeConstants.kIntakePivotID, IntakeConstants.kIntakeMotorCANBus);

        powerRequest = new DutyCycleOut(0);
    }

    public void stopIntakeRollers ()
    {
        powerRequest.Output = 0;

        IntakeRollerMotor1.setControl(powerRequest);
        IntakeRollerMotor1.setControl(powerRequest);
    }

    public void setIntakeRollerPower(double power)
    {
        powerRequest.Output = power;
        IntakeRollerMotor1.setControl(powerRequest);
                        // Likely bad practice to alter one power request multiple times in one method; look in to later
        powerRequest.Output = -power;
        IntakeRollerMotor1.setControl(powerRequest);
    }

    public void setIntakePivotPower(double power)
    {
        powerRequest.Output = power;

        IntakePivotMotor.setControl(powerRequest);
    }

    @Override
    public void periodic()
    {

    }
}
