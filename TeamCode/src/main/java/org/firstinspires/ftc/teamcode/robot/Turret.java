package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 터렛 회전 (Axon MAX MK2, continuous 모드).
 * continuous 모드의 Axon MAX MK2는 각도를 알려 주는 선이 없어서, 코드는 터렛이 지금 몇 도인지 모른다.
 * 그래서 지금은 오퍼레이터가 스틱으로 직접 돌린다 (자동 조준·한계각 없음 → 선이 꼬이지 않게 주의).
 */
@Configurable
public class Turret {
    /** 스틱을 오른쪽으로 밀 때 터렛이 반대로 돌면 true. */
    public static boolean REVERSED = false;
    /** 스틱 끝까지 밀었을 때 파워 (0~1). 너무 빠르면 낮출 것. */
    public static double MAX_POWER = 0.5;
    /** 정밀 조작(버튼) 파워. */
    public static double FINE_POWER = 0.15;

    private final CRServo servo;
    private double power;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, HardwareNames.TURRET);
        servo.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servo.setPower(0);
    }

    /** @param input -1(왼쪽) ~ 1(오른쪽) */
    public void manual(double input) {
        power = Math.max(-1, Math.min(1, input)) * MAX_POWER;
        servo.setPower(power);
    }

    /** @param direction -1, 0, 1 */
    public void fine(int direction) {
        power = direction * FINE_POWER;
        servo.setPower(power);
    }

    public void stop() { power = 0; servo.setPower(0); }
    public double power() { return power; }
}
