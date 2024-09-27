package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.BaseVelocityTelemetry;
import org.firstinspires.ftc.teamcode.components.BasketComponentV1;
import org.firstinspires.ftc.teamcode.components.DrivingBase;
import org.firstinspires.ftc.teamcode.components.HDriveComponentV1;
import org.firstinspires.ftc.teamcode.components.JoystickTelemetry;
import org.firstinspires.ftc.teamcode.components.LinearMotionComponentV1;
import org.firstinspires.ftc.teamcode.components.LinearMotionTelemetry;
import org.firstinspires.ftc.teamcode.components.VisionComponentV1;
import org.firstinspires.ftc.teamcode.utilities.ComponentOpMode;

@TeleOp
@Disabled
public class RobotTestingV4 extends ComponentOpMode {
    @Override
    public void componentsSetup() {
        DrivingBase drivingBase = new DrivingBase(hardwareMap);
        HDriveComponentV1 hDrive = new HDriveComponentV1(drivingBase);

        components.add(hDrive);
        //components.add(new VisionComponentV1(drivingBase, hDrive));
        components.add(new LinearMotionComponentV1());
        components.add(new BasketComponentV1());

        components.add(new BaseVelocityTelemetry());
        components.add(new LinearMotionTelemetry());
        components.add(new JoystickTelemetry());
    }
}
