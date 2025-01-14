package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue; // Correct import
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    private final TalonFX krakenMotor;
    private static final int TICKS_PER_REVOLUTION = 2048; // Number of encoder ticks per revolution

    public Motor(int motorCANID) {
        krakenMotor = new TalonFX(motorCANID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

        krakenMotor.getConfigurator().apply(config);
    }

    public void setMotorSpeed(double speed) {
        krakenMotor.setControl(new DutyCycleOut(speed));
    }

    public void stopMotor() {
        setMotorSpeed(0);
    }

    public void setMotorPosition(double degrees) {
        double targetTicks = (degrees / 360.0) * TICKS_PER_REVOLUTION;
    
        // just finetune
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
    
        krakenMotor.getConfigurator().apply(config);
    
        PositionVoltage positionControl = new PositionVoltage(targetTicks);
        krakenMotor.setControl(positionControl);
    }
    
}
