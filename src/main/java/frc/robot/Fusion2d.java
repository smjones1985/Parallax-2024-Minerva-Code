package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.LinkedList;
import java.util.Queue;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Fusion2d {
    private Pigeon2 pigeon;
    private Pose2d pose = new Pose2d();
    private MovingAverage acc_x = new MovingAverage(5);
    private MovingAverage acc_y = new MovingAverage(5);
    private MovingAverage rot = new MovingAverage(5);
    private double[] linearVelocity = { 0, 0 };
    private double[] linearPosition = { 0, 0 };
    private double[] gravityVector = { 0, 0, 0 };
    private float loopTime = 0.02f;
    private double yawOffset = 0.0d;

    public Fusion2d(Pigeon2 pigeon) {
        this.pigeon = pigeon;
        this.gravityVector[0] = this.pigeon.getGravityVectorX().getValueAsDouble();
        this.gravityVector[1] = this.pigeon.getGravityVectorY().getValueAsDouble();
        this.gravityVector[2] = this.pigeon.getGravityVectorZ().getValueAsDouble();
    }

    public void resetPosition(Rotation2d rotation, Pose2d pose) {
        this.yawOffset = rotation.getDegrees() - this.pose.getRotation().getDegrees();
        this.pose = new Pose2d(pose.getTranslation(), rotation);
    }

    public void resetPosition(Pose2d pose) {
        this.yawOffset = pose.getRotation().getDegrees() - this.pose.getRotation().getDegrees();
        this.pose = pose;
    }

    // s = s + u * dt;
    // u = u + a * dt;

    public void update() {
        // System.out.println("(" + this.pigeon.getGravityVectorX() + ", " +
        // this.pigeon.getGravityVectorZ() + ", "
        // + this.pigeon.getGravityVectorY() + ")");
        // System.out.println("(" + this.pigeon.getAccelerationX() + ", " +
        // this.pigeon.getAccelerationZ() + ", "
        // + this.pigeon.getAccelerationY() + ")");

        acc_x.next(
                this.pigeon.getAccelerationX().getValueAsDouble() - this.gravityVector[0]);
        acc_y.next(
                this.pigeon.getAccelerationY().getValueAsDouble() - this.gravityVector[1]);
        rot.next(this.pigeon.getYaw().getValueAsDouble());

        this.linearVelocity[0] = this.linearVelocity[0] + (acc_x.get()) * 9.80665 * this.loopTime;
        this.linearPosition[0] = this.linearPosition[0] + this.linearVelocity[0] * this.loopTime;

        this.linearVelocity[1] = this.linearVelocity[1] + (acc_y.get()) * 9.80665 * this.loopTime;
        this.linearPosition[1] = this.linearPosition[1] + this.linearVelocity[1] * this.loopTime;

        this.pose = new Pose2d(this.linearPosition[0], this.linearPosition[1],
                Rotation2d.fromDegrees(rot.get() + this.yawOffset));
    }

    public Pose2d getPoseMeters() {
        return this.pose;
    }
}

class MovingAverage {

    private int windowSize;
    private Queue<Double> window;
    private double sum;

    public MovingAverage(int windowSize) {
        this.windowSize = windowSize;
        this.window = new LinkedList<>();
        this.sum = 0.0d;
    }

    public double next(double value) {
        System.out.println(window.size());

        value = MovingAverage.round(value, 2);

        window.add(value);
        sum += value;

        if (window.size() > windowSize) {
            sum -= window.remove();
        }

        return sum / window.size();
    }

    public double get() {
        return sum / window.size();
    }

    /**
     * Round to certain number of decimals
     * 
     * @param d
     * @param decimalPlace
     * @return
     */
    public static double round(double d, int decimalPlace) {
        BigDecimal bd = new BigDecimal(Double.toString(d));
        bd = bd.setScale(decimalPlace, RoundingMode.DOWN);
        return bd.doubleValue();
    }
}
