package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 플라워에서 공을 빼오는 쐐기 (goBILDA 2000-0025-0502, 5-turn 서보).
 * 5-turn 서보는 0~1이 5바퀴(약 1800°)라서 0.01만 바꿔도 약 18° 움직인다. 값을 조금씩 바꿀 것.
 * TODO(25323): 위치는 임시값. Hardware Check로 찾아서 바꿀 것.
 */
@Configurable
public class Wedge {
    /** 쐐기를 넣은(접은) 위치. */
    public static double RETRACTED = 0.5;
    /** 쐐기를 내민(공 빼는) 위치. */
    public static double EXTENDED = 0.55;

    private final Servo servo;
    private boolean extended;

    public Wedge(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, HardwareNames.WEDGE);
    }

    public void extend() { servo.setPosition(EXTENDED); extended = true; }
    public void retract() { servo.setPosition(RETRACTED); extended = false; }
    public boolean isExtended() { return extended; }
}
