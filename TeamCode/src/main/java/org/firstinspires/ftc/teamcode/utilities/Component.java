package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component {
    protected HardwareMap hardwareMap;
    protected GamepadEx driverGamepad, ballerGamepad;
    protected Telemetry telemetry;

    public abstract void initializeComponent();

    public abstract void runLoop();

    public void loadConfig(ComponentConfig config) {
        hardwareMap = config.getHardwareMap();
        driverGamepad = config.getDriverGamepad();
        ballerGamepad = config.getBallerGamepad();
        telemetry = config.getTelemetry();
    }
}
