package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    private final TalonFX krakenMotor;
    private static final int TICKS_PER_REVOLUTION = 2048; // Number of encoder ticks per revolution

    public Motor(int motorCANID) {
        krakenMotor = new TalonFX(motorCANID);

        // Initialize motor settings (e.g., neutral mode, sensor phase)
        krakenMotor.setInverted(false); // Adjust as needed
    }

    public void setMotorSpeed(double speed) {
        krakenMotor.setControl(new DutyCycleOut(speed));
    }

    public void stopMotor() {
        setMotorSpeed(0);
    }

    public void setMotorPosition(double degrees) {
        // Convert degrees to encoder ticks
        double targetTicks = (degrees / 360.0) * TICKS_PER_REVOLUTION;

        // Set position using PositionVoltage control
        krakenMotor.setControl(new PositionVoltage(targetTicks, true));
    }
}
