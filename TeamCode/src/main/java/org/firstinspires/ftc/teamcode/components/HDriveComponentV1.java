package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.IToggle;

public class HDriveComponentV1 extends Component implements IToggle {
    private final DrivingBase drivingBase;

    private ButtonReader hDriveToggle;
    private boolean enabled = true;

    public HDriveComponentV1(DrivingBase drivingBase) {
        this.drivingBase = drivingBase;
    }

    @Override
    public void initializeComponent() {
        hDriveToggle = new ButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
    }

    @Override
    public void runLoop() {
        if (enabled) {
            drivingBase.tankWheels.arcadeDrive(driverGamepad.getLeftY(),
                    driverGamepad.getRightX(), true);
            if (hDriveToggle.isDown()) {
                double leftTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                double rightTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                if (leftTrigger > 0.1 || rightTrigger > 0.1) {
                    drivingBase.hDrive.set(Math.pow(rightTrigger - leftTrigger, 2) * Math.signum(rightTrigger - leftTrigger));
                } else {
                    drivingBase.hDrive.set(Math.pow(driverGamepad.getLeftX(), 2) * Math.signum(driverGamepad.getLeftX()));
                }
                drivingBase.setServos(true);
            } else {
                drivingBase.hDrive.stopMotor();
                drivingBase.setServos(false);
            }
        }
    }

    @Override
    public void enable() {
        enabled = true;
    }

    @Override
    public void disable() {
        enabled = false;
    }
}
