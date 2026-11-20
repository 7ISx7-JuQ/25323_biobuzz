package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 터렛 회전 (Axon MAX MK2, continuous 모드).
 *
 * 피드백 선(Analog Input "turretEnc")이 없으면: 스틱으로만 돌린다. 각도를 모르므로 한계각·가운데 복귀 없음.
 * 피드백 선이 있으면: 서보가 돈 각도를 계속 더해서 터렛 각도를 계산한다.
 *   - INIT 할 때 터렛이 가운데(인테이크 방향)에 있어야 한다 → 그 방향이 0°.
 *   - GEAR_RATIO 를 넣어야 각도가 맞는다 (Turret Ratio Test로 측정). 0이면 각도 기능 꺼짐.
 *   - LIMIT_DEG 를 넣으면 ±LIMIT_DEG 밖으로는 안 돈다 (선 꼬임 방지). 0이면 한계각 꺼짐.
 *   - center() 를 부르면 0°로 돌아간다.
 */
@Configurable
public class Turret {
    /** 스틱을 오른쪽으로 밀 때 터렛이 반대로 돌면 true. */
    public static boolean REVERSED = false;
    /** 스틱 끝까지 밀었을 때 파워 (0~1). */
    public static double MAX_POWER = 0.5;
    /** 정밀 조작(버튼) 파워. */
    public static double FINE_POWER = 0.15;

    // ── 피드백 선이 있을 때만 쓰는 값 ──
    /** 기어비 = 서보 회전수 ÷ 터렛 회전수 (Turret Ratio Test). 0 = 아직 모름 → 각도 기능 꺼짐. */
    public static double GEAR_RATIO = 0;
    /** 스틱을 오른쪽으로 돌렸는데 화면의 터렛 각도가 줄어들면 true. */
    public static boolean FEEDBACK_REVERSED = false;
    /** 서보가 한 바퀴 돌 때 피드백 전압의 최고값 (V). Turret Ratio Test 화면의 "최고 전압"을 보고 맞출 것. */
    public static double FEEDBACK_MAX_V = 3.3;
    /** 가운데(0°)에서 좌우로 이 각도까지만 돈다. 0 = 한계각 꺼짐. 실측 후 넣을 것. */
    public static double LIMIT_DEG = 0;
    /** 가운데 복귀: 오차 1°당 파워. */
    public static double CENTER_KP = 0.01;
    /** 가운데 복귀: 최대 파워. */
    public static double CENTER_MAX_POWER = 0.35;
    /** 가운데 복귀: 이 오차(°) 안에 들어오면 끝. */
    public static double CENTER_TOLERANCE_DEG = 2;

    private final CRServo servo;
    private final AnalogInput feedback;
    private double power;
    private double servoDeg;          // INIT 이후 서보가 돈 누적 각도
    private double lastServoAngle;
    private boolean centering;
    private boolean atLimit;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, HardwareNames.TURRET);
        servo.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servo.setPower(0);
        feedback = hardwareMap.tryGet(AnalogInput.class, HardwareNames.TURRET_FEEDBACK);
        if (feedback != null) lastServoAngle = servoAngle();
    }

    /** 피드백 선이 연결·등록되어 있는지. */
    public boolean hasFeedback() { return feedback != null; }

    /** 각도 기능을 쓸 수 있는지 (피드백 있음 + 기어비 입력됨). */
    public boolean angleKnown() { return feedback != null && GEAR_RATIO > 0; }

    /** 터렛 각도 (°, 0 = INIT 때 방향, + = 스틱 오른쪽 방향). 각도를 모르면 NaN. */
    public double angleDeg() { return angleKnown() ? servoDeg / GEAR_RATIO : Double.NaN; }

    /** 가운데(0°)로 돌아가기 시작. 스틱을 움직이면 취소된다. */
    public void center() { if (angleKnown()) centering = true; }
    public boolean isCentering() { return centering; }
    public boolean atLimit() { return atLimit; }

    /**
     * 매 루프 한 번 호출.
     * @param stick -1(왼쪽) ~ 1(오른쪽), 스틱 값
     * @param fine  -1, 0, 1 (정밀 버튼)
     */
    public void update(double stick, int fine) {
        trackAngle();

        double want;
        if (fine != 0) want = fine * FINE_POWER;
        else want = Math.max(-1, Math.min(1, stick)) * MAX_POWER;

        if (want != 0) centering = false;          // 사람이 조작하면 자동 복귀 취소
        if (centering) {
            double error = -angleDeg();
            if (Math.abs(error) <= CENTER_TOLERANCE_DEG) {
                centering = false;
                want = 0;
            } else {
                want = Math.max(-CENTER_MAX_POWER, Math.min(CENTER_MAX_POWER, error * CENTER_KP));
            }
        }

        atLimit = false;
        if (angleKnown() && LIMIT_DEG > 0) {
            double a = angleDeg();
            if ((a >= LIMIT_DEG && want > 0) || (a <= -LIMIT_DEG && want < 0)) {
                want = 0;
                atLimit = true;
            }
        }

        power = want;
        servo.setPower(power);
    }

    public void stop() {
        centering = false;
        power = 0;
        servo.setPower(0);
    }

    public double power() { return power; }
    /** 피드백 전압 (V). 피드백 없으면 NaN. */
    public double feedbackVoltage() { return feedback != null ? feedback.getVoltage() : Double.NaN; }

    private void trackAngle() {
        if (feedback == null) return;
        double angle = servoAngle();
        double step = angle - lastServoAngle;
        if (step > 180) step -= 360;        // 359° → 1° 처럼 한 바퀴 넘어간 경우
        if (step < -180) step += 360;
        lastServoAngle = angle;
        servoDeg += FEEDBACK_REVERSED ? -step : step;
    }

    /** 피드백 전압 → 서보 각도 (0~360°). */
    private double servoAngle() { return voltageToDeg(feedback.getVoltage()); }

    /** 피드백 전압 → 서보 각도 (0~360°). Turret Ratio Test도 이 계산을 쓴다. */
    public static double voltageToDeg(double volts) {
        double max = FEEDBACK_MAX_V > 0 ? FEEDBACK_MAX_V : 3.3;
        return Math.max(0, Math.min(1, volts / max)) * 360.0;
    }
}
