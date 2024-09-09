package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GravityGainMotor extends Motor {
    private double gravityGain;

    public GravityGainMotor() {
        super();
    }

    public GravityGainMotor(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    public GravityGainMotor(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public GravityGainMotor(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }

    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration());
            motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND + gravityGain);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            motor.setPower(output * error + gravityGain);
        } else {
            motor.setPower(output + gravityGain);
        }
    }

    public void setGravityGain(double value) {
        if (value > 1) {
            gravityGain = 1;
        } else if (value < -1) {
            gravityGain = -1;
        } else {
            gravityGain = value;
        }
    }
}
