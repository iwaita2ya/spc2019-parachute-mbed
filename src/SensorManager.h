//
// Created by Tatsuya Iwai (GreySound) on 6/7/19.
//

#ifndef SPC2019_PARACHUTE_MBED_SENSORMANAGER_H
#define SPC2019_PARACHUTE_MBED_SENSORMANAGER_H

#include <mbed.h>
#include <MS5607I2C.h>
#include "SystemParameters.h"

namespace greysound {

    class SensorManager {

    public:

        MS5607I2C *ms5607;

        int activeTimeMs;
        float currentPressure;      // 現在の気圧

        float currentTemperature;   // 現在の気温
        uint8_t maxAltitude;            // 最大到達高度
        uint8_t altitudeThreshold;      // 状態遷移に必要な高度しきい値
        uint8_t counterThreshold;       // 状態カウンタのしきい値（この値に達したら、その状態が発生したと判断する）
        uint8_t deployParachuteAt;      // パラシュート開放高度(m)（地表高度に加算）
        uint8_t flyingAltitudeCounter;  // 飛行高度到達カウンタ
        uint8_t fallingAltitudeCounter; // 落下検知カウンタ
        uint8_t deployAltitudeCounter;  // パラシュート開放高度到達カウンタ
        uint8_t touchDownCounter;       // 着地判断カウンタ

        // センサの状態管理
        enum SensorState {
            CREATED,        // インスタンス生成完了
            STAND_BY,       // データ取得準備完了
            ACTIVE,         // データ取得中
            BUSY,           // データ更新中
            BEGIN_FAILED    // データ取得開始失敗
        };

        /**
         * コンストラクタ
         * @param _sda
         * @param _scl
         * @param _agAddr
         * @param _mAddr
         */
        SensorManager(PinName _sda, PinName _scl, uint8_t _agAddr, uint8_t _mAddr, SystemParameters *systemParams)
        {
            // インスタンス初期化
            params = systemParams;
            ms5607  = new MS5607I2C(_sda, _scl, true); // sda, scl csb(1:0xEC 0:0xEE)
            ms5607->setPressureAtSeaLevel(params->pressureAtSeaLevel);

            activeTimer = new Timer();


            // 変数初期化
            currentPressure = 0.0f;
            currentTemperature = 0.0f;
            maxAltitude = 0;
            altitudeThreshold = systemParams->altitudeThreshold;
            counterThreshold  = systemParams->counterThreshold;
            deployParachuteAt = systemParams->deployParachuteAt;

            // カウンタリセット
            //resetCounters();
            activeTimeMs = 0;
            flyingAltitudeCounter  = 0;
            fallingAltitudeCounter = 0;
            deployAltitudeCounter  = 0;
            touchDownCounter       = 0;

            // CREATED 状態に遷移
            currentState = CREATED;
        }

        /**
         * デストラクタ
         */
        ~SensorManager(){
            // stop all activities
            end();

            // release object
            delete params;
            delete ms5607;
            delete activeTimer;
        }

        /**
         * 現在高度のサンプリングを行い、その結果を地表高度として設定する。
         * MEMO: 機体が地上に設置されている状態で呼ばれる前提なので、飛行中の実行はNGです。
         * @return
         */
        uint8_t calculateGroundAltitude()
        {
            const uint8_t numberOfSamples = 8;
            float pressureSamples = 0.0;

            // 現在高度をサンプリングする
            for (uint8_t i = 0; i<numberOfSamples; i++) {
                pressureSamples += ms5607->getPressure();
                wait(0.1);
            }

            // 現在の気圧を平均値から求める
            currentPressure = (pressureSamples / numberOfSamples);

            // 現在高度を気圧から求める
            params->currentAltitude = (uint8_t)(ms5607->getAltitude((int)currentPressure));

            // 現在高度を地表高度として設定する
            params->groundAltitude = params->currentAltitude;

            return 0;
        }

        uint8_t begin()
        {
            // まず STAND_BY の状態まで遷移する
            uint8_t result = 0;
            switch (currentState) {
                case CREATED:
                case BEGIN_FAILED:
                    result = this->getStandBy();
                    break;

                case ACTIVE: //MEMO: ACTIVEの場合は処理をしないほうが良い？（電源喪失時等を考慮して）
                case BUSY:
                    this->end(); // 既に実行中の場合は一旦停止
                    break;

                case STAND_BY:
                default:
                    break;
            }

            // error
            if(result != 0) {
                currentState = BEGIN_FAILED;
                return 1;
            }

            // カウンタリセット
            resetCounters();

            // タイマー開始
            activeTimer->start();

            // ACTIVE 状態に遷移
            currentState = ACTIVE;

            return 0;
        }

        // データを読み取る
        uint8_t update()
        {
            // すでに処理が走っている場合はスキップ（処理自体は成功とする）
            if (currentState == BUSY) {
                return 0;
            }

            // BUSY 状態に遷移
            currentState = BUSY;

            // current time (ms)
            activeTimeMs = activeTimer->read_ms();

            // 現在高度を取得
            currentPressure = ms5607->getPressure();
            params->currentAltitude = (uint8_t) ms5607->getAltitude((int)currentPressure);

            // 現在高度が最大到達高度を上回っていたら更新
            if (params->currentAltitude > maxAltitude) {
                maxAltitude = (uint8_t) params->currentAltitude;
            }

            // 飛行状態チェック：現在高度が規定値を上回っている場合はカウントアップ
            if(params->currentAltitude > (params->groundAltitude + altitudeThreshold) && flyingAltitudeCounter < counterThreshold) {
                flyingAltitudeCounter++;
            }

            // 落下状態チェック：最大到達高度が規定値を下回り始めたらカウントアップ
            if((params->currentAltitude + altitudeThreshold) < maxAltitude &&  fallingAltitudeCounter < counterThreshold) {
                fallingAltitudeCounter++;
            }

            // パラシュート開放チェック：落下状態で、現在高度がパラシュート開放高度を下回った場合はカウントアップ
            if(isFalling() && params->currentAltitude < (params->groundAltitude + deployParachuteAt) && deployAltitudeCounter < counterThreshold) {
                deployAltitudeCounter++;
            }

            // 着地状態チェック：飛行状態で、現在高度が既定値を下回っていた場合はカウントアップ
            if(isFalling() && params->currentAltitude < (params->groundAltitude + altitudeThreshold) && touchDownCounter < counterThreshold) {
                touchDownCounter++;
            }

            // ACTIVE 状態に戻る
            currentState = ACTIVE;

            return 0;
        }

        /**
         * ステータスに関係なく強制的にセンサの値を更新する
         * (速度が要求されない待機状態での使用を想定)
         */
        void updateForced()
        {
            // ms5607
            currentPressure = ms5607->getPressure();
            params->currentAltitude = (uint8_t) ms5607->getAltitude((int)currentPressure);
            currentTemperature = ms5607->getTemperature();
        }

        void end()
        {
            activeTimer->stop();
            currentState = STAND_BY;
        }

        /**
         * 飛行状態か？
         * @return
         */
        bool isFlying()
        {
            return (flyingAltitudeCounter >= counterThreshold);
        }

        /**
         * 落下状態か？
         * @return
         */
        bool isFalling()
        {
            return (fallingAltitudeCounter >= counterThreshold);
        }

        /**
         * パラシュートを開放してもよいか？
         * @return
         */
        bool isOkToDeployParachute()
        {
            return (deployAltitudeCounter >= counterThreshold);
        }

        /**
         * 着地状態か？
         * @return
         */
        bool isTouchDown()
        {
            return (touchDownCounter >= counterThreshold);
        }

        /**
         * ステータス状態を返す
         * @return
         */
        SensorState getCurrentState()
        {
            return currentState;
        }

    protected:

    private:
        SensorState currentState;
        Timer *activeTimer;
        SystemParameters *params;

        /**
         * カウンタ群リセット
         */
        void resetCounters() {
            activeTimeMs = 0;
            flyingAltitudeCounter  = 0;
            fallingAltitudeCounter = 0;
            deployAltitudeCounter  = 0;
            touchDownCounter       = 0;
        }

        /**
         * データ取得のための準備を行う
         */
        uint8_t getStandBy()
        {

            // カウンタリセット
            resetCounters();

            // スタンバイ状態に遷移
            currentState = STAND_BY;

            return 0;
        }

    };

}

#endif //SPC2019_PARACHUTE_MBED_SENSORMANAGER_H
