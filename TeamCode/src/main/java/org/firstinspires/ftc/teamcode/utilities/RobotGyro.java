package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotGyro {

    private IMU revIMU;

    private int multiplier;

    public RobotGyro(HardwareMap hw, String imuName) {
        revIMU = hw.get(IMU.class, imuName);
        multiplier = 1;
    }

    public RobotGyro(HardwareMap hw) {
        this(hw, "imu");
    }

    public void init() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        init(parameters);
    }

    public void init(IMU.Parameters parameters) {
        revIMU.initialize(parameters);
    }

    public void invertGyro() {
        multiplier *= -1;
    }

    public double getRelativeHeading() {
        YawPitchRollAngles orientation = revIMU.getRobotYawPitchRollAngles();

        return orientation.getYaw(AngleUnit.DEGREES) * multiplier;
    }

    public double[] getAngles() {
        YawPitchRollAngles orientation = revIMU.getRobotYawPitchRollAngles();

        return new double[]{orientation.getPitch(AngleUnit.DEGREES), orientation.getRoll(AngleUnit.DEGREES), orientation.getYaw(AngleUnit.DEGREES)};
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRelativeHeading());
    }

    public void resetYaw() {
        revIMU.resetYaw();
    }

    public void disable() {
        revIMU.close();
    }

    public IMU getRevIMU() {
        return revIMU;
    }

}