package frc.robot.subsystems.ArmSubsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class PitchMotorSubsystemTest {

    private PitchMotorSubsystem pitchMotorSubsystem;
  
    private CANSparkMax pitchMotor = mock(CANSparkMax.class);

    private PIDController pidController = mock(PIDController.class);

    private PIDController fasterPidController = mock(PIDController.class);

    private AnalogEncoder pitchMotorEncoder = mock(AnalogEncoder.class);

    private RelativeEncoder relativeEncoder = mock(RelativeEncoder.class);

    private ShuffleboardTab encoderTab = mock(ShuffleboardTab.class);

    private SimpleWidget pitchMotorSpeed = mock(SimpleWidget.class);

    private GenericEntry entry = mock(GenericEntry.class);

    @BeforeEach
    public void setup() {
        when(pitchMotor.getEncoder()).thenReturn(relativeEncoder);
        when(pitchMotorSpeed.getEntry()).thenReturn(entry);
        when(encoderTab.add("Encoder Voltage", 0.0)).thenReturn(mock(SimpleWidget.class));
        when(encoderTab.add("Encoder Degrees", 0.0)).thenReturn(mock(SimpleWidget.class));
        when(encoderTab.add("Pitch Motor Speed", 0.0)).thenReturn(pitchMotorSpeed);
        when(encoderTab.add("Internal Encoder Position", 0.0)).thenReturn(mock(SimpleWidget.class));
        pitchMotorSubsystem = new PitchMotorSubsystem(pitchMotor, pidController, fasterPidController, pitchMotorEncoder, encoderTab);
    }

    @Test
    public void testAddBaseIdleForce() {
        var motorSpeed = 0.5;
        var result = pitchMotorSubsystem.addBaseIdleForce(motorSpeed);
        assertTrue(result >= -1.0 && result <= 1.0, "Motor speed should be clamped between -1.0 and 1.0");
    }

    @Test
    public void testRunPitchMotor() {
        var motorSpeed = 0.8;
        pitchMotorSubsystem.runPitchMotor(motorSpeed);
    }

    //these tests highlight that the constant aim always flips the current setting. Was this desired behavior?
    @Test
    public void testConstantAimSetTrue() {
        pitchMotorSubsystem.setConstantAim(true);
        pitchMotorSubsystem.constantAim();
        assertFalse(pitchMotorSubsystem.getConstantAim());
    }

    
    @Test
    public void testConstantAimSetFalse() {
        pitchMotorSubsystem.setConstantAim(false);
        pitchMotorSubsystem.constantAim();
        assertTrue(pitchMotorSubsystem.getConstantAim());
    }


    
    @Test
    public void testRunPitchMotorWithKP() {
        var angleDeg = 89.0;
        pitchMotorSubsystem.runPitchMotorWithKP(angleDeg);
    }

    @Test
    public void testRunPitchMotorWithKP_clamps() {
        var angleDeg = 91.0;
        pitchMotorSubsystem.runPitchMotorWithKP(angleDeg);
    }
}
