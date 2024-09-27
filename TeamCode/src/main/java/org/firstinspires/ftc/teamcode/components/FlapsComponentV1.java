package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class FlapsComponentV1 extends Component {
    private MotorGroup flaps;
    private ToggleButtonReader flapsReader;
    private ButtonReader up, down;
    private Motor.RunMode current;

    @Override
    public void initializeComponent() {
        Motor flapsLeft = new Motor(hardwareMap, RobotConstants.FLAPS_LEFT);
        flapsLeft.setInverted(true);
        Motor flapsRight = new Motor(hardwareMap, RobotConstants.FLAPS_RIGHT);

        flaps = new MotorGroup(flapsLeft, flapsRight);
        flaps.setRunMode(Motor.RunMode.PositionControl);
        flaps.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flaps.resetEncoder();

        flapsReader = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.B);
        up = new ButtonReader(driverGamepad, GamepadKeys.Button.DPAD_LEFT);
        down = new ButtonReader(driverGamepad, GamepadKeys.Button.DPAD_RIGHT);

        current = Motor.RunMode.PositionControl;

        setServos(true);
    }

    @Override
    public void runLoop() {
        flapsReader.readValue();

        if (flapsReader.isDown()) {
            current = Motor.RunMode.PositionControl;
            flaps.setRunMode(current);
        }

        if (flapsReader.getState()) {
            setServos(false);
        } else {
            setServos(true);
        }

        if (down.isDown()) {
            current = Motor.RunMode.RawPower;
            flaps.setRunMode(current);
            flaps.set(0.4);
        } else if (up.isDown()) {
            current = Motor.RunMode.RawPower;
            flaps.setRunMode(current);
            flaps.set(-0.4);
        } else if (current == Motor.RunMode.RawPower) {
            flaps.set(0);
        }

        if (current == Motor.RunMode.PositionControl) {
            flaps.set(0.013);
        }

        telemetry.addLine();
        telemetry.addLine("Flaps Telemetry:");
        telemetry.addData("Flap pos", flaps.getPositions().get(0));
    }

    private void setServos(boolean up) {
        if (up) {
            flaps.setTargetPosition(2);
        } else {
            flaps.setTargetPosition(70);
        }
    }
}
