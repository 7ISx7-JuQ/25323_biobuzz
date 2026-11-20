package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * 플라이휠 슈터 (goBILDA 6000rpm, 엔코더 28 ticks/회전 → 최고 약 2800 ticks/s).
 * 속도 제어(PIDF)로 목표 속도를 유지하고, 배터리 전압이 떨어지면 F를 자동으로 올린다.
 * TODO(25323): 속도와 PIDF 값은 임시값. Tuning 페이지의 Shooter PIDF Tuner로 찾을 것.
 */
@Configurable
public class Shooter {
    public enum Mode { OFF, IDLE, SHOOT }

    /** 앞으로 쏠 때 Hardware Check에서 속도가 음수로 나오면 true. */
    public static boolean REVERSED = false;
    /** 발사 속도 (ticks/s). 최고 약 2800. */
    public static double SHOOT_VELOCITY = 1500;
    /** 평소 예열 속도 (ticks/s). 발사 버튼을 누르면 SHOOT_VELOCITY까지 빨리 올라간다. */
    public static double IDLE_VELOCITY = 800;
    /** 목표 속도에서 이만큼 모자라도 "준비됨"으로 본다 (ticks/s). */
    public static double READY_TOLERANCE = 50;

    /** 속도 PIDF. F는 12V 기준 값 (32767 / 2800 ≈ 11.7). */
    public static double P = 30, I = 0, D = 0, F = 11.7;

    private static final long VOLTAGE_REFRESH_NS = 500_000_000L;

    private final DcMotorEx motor;
    private final HardwareMap hardwareMap;
    private Mode mode = Mode.OFF;
    private long nextVoltageNs;
    private double voltage = 12;
    private double appliedP = Double.NaN, appliedI = Double.NaN, appliedD = Double.NaN, appliedF = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, HardwareNames.SHOOTER);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // 멈출 때 플라이휠이 천천히 서도록
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(0);
    }

    public void setMode(Mode mode) { this.mode = mode; }
    public Mode mode() { return mode; }

    /** 매 루프 호출. */
    public void update() {
        refreshPidf();
        motor.setVelocity(targetVelocity());
    }

    public double targetVelocity() {
        switch (mode) {
            case SHOOT: return SHOOT_VELOCITY;
            case IDLE: return IDLE_VELOCITY;
            default: return 0;
        }
    }

    public double velocity() { return motor.getVelocity(); }

    /** 발사 속도에 도달했는지. */
    public boolean atShootSpeed() { return velocity() >= SHOOT_VELOCITY - READY_TOLERANCE; }

    public void stop() {
        mode = Mode.OFF;
        motor.setVelocity(0);
    }

    private void refreshPidf() {
        long now = System.nanoTime();
        if (now >= nextVoltageNs) {
            double lowest = Double.POSITIVE_INFINITY;
            for (VoltageSensor s : hardwareMap.voltageSensor) {
                double v = s.getVoltage();
                if (v > 0 && v < lowest) lowest = v;
            }
            if (!Double.isInfinite(lowest)) voltage = lowest;
            nextVoltageNs = now + VOLTAGE_REFRESH_NS;
        }
        double f = F * (12.0 / voltage);
        if (P != appliedP || I != appliedI || D != appliedD || Math.abs(f - appliedF) >= 0.01) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, f));
            appliedP = P; appliedI = I; appliedD = D; appliedF = f;
        }
    }
}
