//
// Created by Tatsuya Iwai (GreySound) on 19/06/17.
//

#ifndef SPC2019_PARACHUTE_MBED_SYSTEM_PARAMETERS_H
#define SPC2019_PARACHUTE_MBED_SYSTEM_PARAMETERS_H

struct SystemParameters {
    uint8_t statusFlags;
    float pressureAtSeaLevel;   // 海抜0mの大気圧
    uint16_t groundAltitude;    // 地表の高度
    uint16_t currentAltitude;   // 現在の高度
    uint8_t deployParachuteAt;  // パラシュート開放高度(地表高度に加算する値)
    uint8_t counterThreshold;   // 状態カウンタのしきい値（この回数に達したら、その状態が発生したと判断する）
    uint8_t altitudeThreshold;  // 状態遷移に必要な高度しきい値
    float openServoPeriod;      // サーボ開 duty比 (0-1.0f)
    float closeServoPeriod;     // サーボ閉 duty比 (0-1.0f)
    uint8_t enableLogging;      // ロギング設定 (0x00:無効 0x01:有効)
    time_t logStartTime;        // ロギング開始時刻(RTCから取得する）
    uint16_t logPointer;        // 最終ログ格納アドレス (0x0020-0x0800)
};

#endif //SPC2019_PARACHUTE_MBED_SYSTEM_PARAMETERS_H
