package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    private final TalonFX krakenMotor;

    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

    public Motor(int motorCANID) {
        krakenMotor = new TalonFX(motorCANID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure PID gains for Motion Magic®
        config.Slot0.kP = 4.8; // Example value, tune as needed
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kS = 0.25; // Static friction compensation
        config.Slot0.kV = 0.12; // Velocity feedforward
        config.Slot0.kA = 0.01; // Acceleration feedforward

        // Configure Motion Magic® parameters
        config.MotionMagic.MotionMagicCruiseVelocity = 80; // Units: rotations/sec
        config.MotionMagic.MotionMagicAcceleration = 160; // Units: rotations/sec^2
        config.MotionMagic.MotionMagicJerk = 1600; // Units: rotations/sec^3

        // Apply configurations with retries
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = krakenMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Motor configuration failed: " + status);
        }

        krakenMotor.setPosition(0); // Start at 0
    }

    public void moveToPosition(double rotations) {
        krakenMotor.setControl(motionMagicControl.withPosition(rotations));
    }

    public void stopMotor() {
        krakenMotor.setControl(new MotionMagicVoltage(0)); // Set to zero voltage
    }
}