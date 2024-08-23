package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * An implementation of the Motor class adapted to the needs of our robot's linear motion
 */
public class LinearMotionMotor extends Motor {
    protected PIDController positionController = new PIDController(1, 0, 0);
    protected double kg;

    public LinearMotionMotor() {
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public LinearMotionMotor(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motor.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public LinearMotionMotor(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        motor = hMap.get(DcMotor.class, id);
        encoder = new Encoder(motor::getCurrentPosition);

        runmode = RunMode.RawPower;
        type = gobildaType;

        ACHIEVABLE_MAX_TICKS_PER_SECOND = gobildaType.getAchievableMaxTicksPerSecond();
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public LinearMotionMotor(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        this(hMap, id, GoBILDA.NONE);

        MotorConfigurationType type = motor.getMotorType().clone();
        type.setMaxRPM(rpm);
        type.setTicksPerRev(cpr);
        motor.setMotorType(type);

        ACHIEVABLE_MAX_TICKS_PER_SECOND = cpr * rpm / 60;
    }

    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration());
            motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition());
            motor.setPower(output * error + kg);
        } else {
            motor.setPower(output);
        }
    }

    public void setPositionCoefficients(double kp, double ki, double kd) {
        positionController.setPID(kp, ki, kd);
    }

    public void setGravityGain(double kg) {
        this.kg = kg;
    }

    @Override
    public void setPositionTolerance(double tolerance) {
        positionController.setTolerance(tolerance);
    }

    public double getTargetPosition() { return positionController.getSetPoint(); }

    @Override
    public void setTargetPosition(int target) {
        positionController.setSetPoint(target);
    }

    @Override
    public boolean atTargetPosition() {
        return positionController.atSetPoint();
    }

}
