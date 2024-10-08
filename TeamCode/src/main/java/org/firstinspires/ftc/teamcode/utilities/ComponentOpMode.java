package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.HashSet;

public abstract class ComponentOpMode extends LinearOpMode {
    protected final HashSet<Component> components = new HashSet<>();

    @Override
    public void runOpMode() {
        componentsSetup();
        initialize();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                for (Component component : components) {
                    component.runLoop();
                }

                sleep(20);
                telemetry.update();
            }
        }
    }

    private void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx ballerGamepad = new GamepadEx(gamepad2);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        ComponentConfig config = new ComponentConfig(hardwareMap, driverGamepad, ballerGamepad, telemetry, this);

        for (Component component : components) {
            component.loadConfig(config);
            component.initializeComponent();
        }
    }

    @SuppressWarnings("unchecked")
    public <T> T getComponentOfType(Class<T> componentType) {
        for (Component component : components) {
            if (componentType.isInstance(component)) {
                return (T) component;
            }
        }

        return null;
    }

    public abstract void componentsSetup();
}
