package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.IToggle;

public class HDriveComponentV1 extends Component implements IToggle {
    private final DrivingBase drivingBase;

    private ToggleButtonReader hDriveToggle;
    private boolean enabled = true;

    public HDriveComponentV1(DrivingBase drivingBase) {
        this.drivingBase = drivingBase;
    }

    @Override
    public void initializeComponent() {
        hDriveToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
    }

    @Override
    public void runLoop() {
        if (enabled) {
            drivingBase.tankWheels.arcadeDrive(driverGamepad.getLeftY(),
                    driverGamepad.getRightX());
            hDriveToggle.readValue();
            if (hDriveToggle.getState()) {
                drivingBase.hDrive.set(driverGamepad.getLeftX());
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
