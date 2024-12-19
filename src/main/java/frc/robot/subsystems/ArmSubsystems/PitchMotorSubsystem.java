package frc.robot.subsystems.ArmSubsystems;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.ArmMotorsConstants.PitchMotor;

public class PitchMotorSubsystem extends SubsystemBase {
    private final CANSparkMax pitchMotor;
    private final PIDController pitchPIDController;
    private final PIDController fasterPitchPIDController;
    private final AnalogEncoder pitchMotorEncoder;
    private final ShuffleboardTab encoderTab;
    private final GenericEntry internalEncoderPosition;
    private final GenericEntry encoderVoltage;
    private final GenericEntry encoderDeg;
    private final GenericEntry pitchMotorSpeed;

    private double baseIdleForce;
    private boolean constantAim;

    public PitchMotorSubsystem() {
        this(new CANSparkMax(PitchMotor.kPitchMotorId, MotorType.kBrushless), 
             new PIDController(PitchMotor.kPitchMotorKP, 0, 0), 
             new PIDController(0.1, 0.1, 0), 
             new AnalogEncoder(ArmMotorsConstants.PitchMotor.kPitchEncoderId),
             Shuffleboard.getTab("Absolute Encoder"));
    }

    public PitchMotorSubsystem(CANSparkMax motor, PIDController pidController, PIDController fasterPidController, AnalogEncoder encoder, ShuffleboardTab shuffleboardTab) {
        pitchMotor = motor;
        pitchPIDController = pidController;
        fasterPitchPIDController = fasterPidController;
        pitchMotorEncoder = encoder;
        encoderTab = shuffleboardTab;

        // make sure all of them have the same settings in case we grabbed one with
        // presets
        pitchMotor.restoreFactoryDefaults();
        constantAim = false;

        configureMotor();
        /* Shuffleboard */
        encoderVoltage = encoderTab.add("Encoder Voltage", 0.0).getEntry();
        encoderDeg = encoderTab.add("Encoder Degrees", 0.0).getEntry();
        pitchMotorSpeed = encoderTab.add("Pitch Motor Speed", 0.0).getEntry();
        internalEncoderPosition = encoderTab.add("Internal Encoder Position", 0.0).getEntry();
    }

    /**
     * Configures motor settings like idle mode, current limit, soft limits, and encoder scaling.
     */
    private void configureMotor() {
        pitchMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        pitchMotor.setSmartCurrentLimit(39);
        pitchMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) PitchMotor.kPitchEncoderReverseLimit);
        pitchMotor.setSoftLimit(SoftLimitDirection.kForward, (float) PitchMotor.kPitchEncoderForwardLimit);
        pitchMotorEncoder.setDistancePerRotation(360);
        pitchMotor.getEncoder().setPositionConversionFactor(PitchMotor.kPitchInternalEncoderConversionFactor);
        pitchMotor.getEncoder().setPosition(getEncoderDeg());
    }


    @Override
    public void periodic() {

        // To prevent the arm from falling while idling, we add a base force that
        // prevents the arm from falling, this should always be added to any movement
        // and be clamped to prevent values that are too high. This basically negates
        // gravity.
        baseIdleForce = PitchMotor.kPitchBaseIdleForce
                * Math.sin((getEncoderDeg() / 360) * (2 * Math.PI));

        /* Shuffleboard */

        // `getAbsolutePosition()` is the *absolute* position of the encoder, no
        // rollovers, no offset.
        encoderVoltage.setDouble(pitchMotorEncoder.getAbsolutePosition());
        // `getDistance()` is the position of the encoder scaled by the distance per
        // rotation, and does have rollovers.
        encoderDeg.setDouble(getEncoderDeg());

        internalEncoderPosition.setDouble(pitchMotor.getEncoder().getPosition());

    }

    public void constantAim(){
        constantAim = !constantAim;
    }

    public boolean getConstantAim(){
        return constantAim;
    }

    public void setConstantAim(boolean constantAim){
        this.constantAim = constantAim;
    }

    /**
     * Applies the force required to fight gravity to the motor speed
     * This should not be used directly, but runPitchMotor(double motorSpeed)
     * already adds this
     * 
     * @param motorSpeed
     * @return motorSpeed + force to fight gravity
     */
    protected double addBaseIdleForce(double motorSpeed) {
        // clamps it between -1 and 1
        return clamp(motorSpeed + this.baseIdleForce, -1.0, 1.0);
    }

    /**
     * Run the pitch motor, but apply the force to fight gravity at <i>the same time!</i>
     * 
     * @param motorSpeed speed to run the pitch motor at (between -1 & 1)
     */
    public void runPitchMotor(double motorSpeed) {
        motorSpeed = addBaseIdleForce(motorSpeed);

        // The speed that the speed controller is applying to the motor.
        pitchMotorSpeed.setDouble(motorSpeed);
        pitchMotor.set(motorSpeed);
    }

    /**
     * This <i>also</i> moves the pitch motor, but does <b>not apply the force to fight gravity</b>.
     * This might even apply a downward force (for intaking)
     * 
     * @param motorSpeed speed to run the pitch motor at (between -1 & 1)
     * @param withoutKP does not matter what it is, as this just differs it from the last method
     */
    public void runPitchMotor(double motorSpeed, boolean withoutKP) {
        motorSpeed -= 0.15;
        // shuffleboard
        pitchMotorSpeed.setDouble(motorSpeed);
        // running it
        pitchMotor.set(motorSpeed);
    }

    /**
     * Gets the position of the <i>external, absolute</i> encoder in degrees
     * 
     * @return the encoder's rotation
     */
    public double getEncoderDeg() {
        return (pitchMotorEncoder.getDistance() + PitchMotor.kPitchEncoderOffset);
    }

    /**
     * Resets the position of the <i>external, absolute</i> encoder
     */
    public void resetEncoder() {
        pitchMotorEncoder.reset();
    }

    /**
     * Uses a PID Controller to move the arm (in the pitch way (up and down)) to a set angle
     * 
     * @param angleDeg the angle to move the arm to
     */
    public void runPitchMotorWithKP(double angleDeg) {
        angleDeg = clamp(angleDeg, -10, PitchMotor.kPitchMotorAmpPresetAngle); // Prevents the motor from moving out of range and breaking itself
        double speed = -(pitchPIDController.calculate(getEncoderDeg(), angleDeg));
        runPitchMotor(speed *= 0.1);
    }

    /**
     * A Tweaked version of the runPitchMotorWithKP method, but with lower P and higher I values
     * 
     * @param angleDeg the angle to move the arm to
     */
    public void runPitchMotorWithFasterKP(double angleDeg) {
        angleDeg = clamp(angleDeg, -10, 90); // Prevents the motor from moving out of range and breaking itself
        double speed = -(fasterPitchPIDController.calculate(getEncoderDeg(), angleDeg));
        runPitchMotor(speed *= 0.1);
    }

    /**
     * A nifty little method to clamp a value between a min and max value
     * 
     * @param value the value to be clamped
     * @param min the min to be clamped above
     * @param max the max to be clamped below
     * @return the value clamped between the max and min
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
