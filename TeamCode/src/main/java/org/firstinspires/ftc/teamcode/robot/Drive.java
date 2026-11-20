package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * 메카넘 필드 기준 주행 (허브 내장 IMU 사용).
 * 모터 방향은 Constants.driveConfig 한 곳에서 가져온다 (Mecanum Tuner 결과를 거기에만 넣으면 됨).
 */
@Configurable
public class Drive {
    /** 허브 로고가 향하는 방향. 허브를 로봇에 단 모양에 맞출 것 (Hardware Check의 IMU 화면으로 확인). */
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    /** 허브 USB 포트가 향하는 방향. */
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    /** 옆으로 갈 때 덜 가는 것 보정 (메카넘 특성). 1.0 = 보정 없음. */
    public static double STRAFE_CORRECTION = 1.1;
    /** 느린 모드 속도 배율 (0~1). */
    public static double SLOW_SCALE = 0.4;
    /** 스틱이 이 값보다 작게 움직이면 0으로 본다. */
    public static double STICK_DEADBAND = 0.05;

    private final DcMotor fl, fr, rl, rr;
    private final IMU imu;
    private boolean fieldCentric = true;

    public Drive(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_LEFT);
        fr = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_RIGHT);
        rl = hardwareMap.get(DcMotor.class, HardwareNames.BACK_LEFT);
        rr = hardwareMap.get(DcMotor.class, HardwareNames.BACK_RIGHT);

        fl.setDirection(Constants.driveConfig.frontLeftDirection.get());
        fr.setDirection(Constants.driveConfig.frontRightDirection.get());
        rl.setDirection(Constants.driveConfig.backLeftDirection.get());
        rr.setDirection(Constants.driveConfig.backRightDirection.get());

        for (DcMotor m : new DcMotor[]{fl, fr, rl, rr}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, HardwareNames.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        imu.resetYaw();
    }

    /** 지금 로봇이 보는 방향을 "앞"으로 정한다 (드라이버 쪽에서 로봇을 경기장 안쪽으로 향하게 놓고 누를 것). */
    public void resetHeading() { imu.resetYaw(); }

    public void setFieldCentric(boolean on) { fieldCentric = on; }
    public boolean isFieldCentric() { return fieldCentric; }

    /** 로봇 방향 (도, 반시계 = +). */
    public double headingDeg() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }

    /**
     * @param forward 앞(+)/뒤(-), 보통 -gamepad.left_stick_y
     * @param strafe  오른쪽(+)/왼쪽(-), 보통 gamepad.left_stick_x
     * @param turn    시계(+)/반시계(-), 보통 gamepad.right_stick_x
     * @param slow    true면 SLOW_SCALE 배로 느리게
     */
    public void drive(double forward, double strafe, double turn, boolean slow) {
        forward = deadband(forward);
        strafe = deadband(strafe);
        turn = deadband(turn);

        double x = strafe, y = forward;
        if (fieldCentric) {
            double h = Math.toRadians(headingDeg());
            // 경기장 기준 입력을 로봇 기준으로 돌린다
            x = strafe * Math.cos(-h) - forward * Math.sin(-h);
            y = strafe * Math.sin(-h) + forward * Math.cos(-h);
        }
        x *= STRAFE_CORRECTION;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double scale = slow ? SLOW_SCALE : 1.0;
        fl.setPower((y + x + turn) / denom * scale);
        rl.setPower((y - x + turn) / denom * scale);
        fr.setPower((y - x - turn) / denom * scale);
        rr.setPower((y + x - turn) / denom * scale);
    }

    public void stop() { drive(0, 0, 0, false); }

    private static double deadband(double v) { return Math.abs(v) < STICK_DEADBAND ? 0 : v; }
}
