//
// Created by Tatsuya Iwai (GreySound) on 19/06/17.
//

#ifndef SPC2019_PARACHUTE_MBED_SYSTEMPARAMETERS_H
#define SPC2019_PARACHUTE_MBED_SYSTEMPARAMETERS_H

struct SystemParameters {
    uint8_t statusFlags;
    float pressureAtSeaLevel;   // 海抜0mの大気圧
    uint8_t groundAltitude;     // 地表の高度
    uint8_t counterThreshold;   // 状態カウンタのしきい値（この値に達したら、その状態が発生したと判断する）
    uint8_t altitudeThreshold;  // 状態遷移に必要な高度しきい値
    uint8_t deployParachuteAt;  // パラシュート開放高度(m)
    float servoPeriod;
    float openServoDuty;
    float closeServoDuty;
    uint8_t enableLogging;
    time_t logStartTime;
    uint16_t lastLogAddress;
};

#endif //SPC2019_PARACHUTE_MBED_SYSTEMPARAMETERS_H
