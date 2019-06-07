//
// Created by iwait on 6/7/19.
//

#ifndef SPC2019_PARACHUTE_MBED_SENSORMANAGER_H
#define SPC2019_PARACHUTE_MBED_SENSORMANAGER_H

#include <mbed.h>
#include <MS5607I2C.h>

#define POINT_THRESHOLD 3
#define DEPLOY_PARACHUTE_AT 30.0f   //MEMO: パラシュート開放高度(m)（地表高度に加算）

namespace greysound {

class SensorManager {


    public:

        MS5607I2C *ms5607;

        int activeTimeMs;
        float currentPressure;
        float currentTemperature;
        float currentAltitude;
        float altitudeToDeploy;
        uint8_t aboveThePointCounter; // 規定高度到達カウンタ

        // センサのステータス
        enum SensorState {
            CREATED,
            STAND_BY,
            ACTIVE,
            BUSY,
            BEGIN_FAILED
        };

        /**
         * コンストラクタ
         * @param _sda
         * @param _scl
         * @param _agAddr
         * @param _mAddr
         */
        SensorManager(PinName _sda, PinName _scl, uint8_t _agAddr, uint8_t _mAddr)
        {
            ms5607  = new MS5607I2C(_sda, _scl, true); // sda, scl csb(1:0xEC 0:0xEE)

            activeTimeMs = 0;
            currentPressure = 0.0f;
            currentTemperature = 0.0f;
            currentAltitude = 0.0f;
            altitudeToDeploy = 0.0f;
            aboveThePointCounter = 0;

            currentState = CREATED;

        }

        ~SensorManager(){
            // stop all activities
            end();

            // メモリ解放
            delete ms5607;
        }

        void init()
        {
            // store current altitude
            setAltitudeToDeploy();
            aboveThePointCounter = 0;

            currentState = STAND_BY;
        }

        /**
         * 落下地点の高度を変数に保存する
         * @return
         */
        float setAltitudeToDeploy()
        {
            const uint8_t numberOfSamples = 16;
            altitudeToDeploy = 0.0f;

            // serial out ms5607 value (debug only)
            for (uint8_t i = 0; i<numberOfSamples; i++) {
                altitudeToDeploy += ms5607->getAltitude();
            }

            // 現在の高度の平均値にしきい値を加算した値を格納する
            altitudeToDeploy = (altitudeToDeploy / numberOfSamples) + DEPLOY_PARACHUTE_AT; //TODO: DEPLOY_PARACHUTE_AT を廃止して変数を使用する

            return altitudeToDeploy;
        }

        uint8_t begin()
        {
            // prepare sensor starting
            switch (currentState) {
                case CREATED:
                case BEGIN_FAILED:
                    this->init();
                    break;

                case ACTIVE:
                case BUSY:
                    this->end();
                    break;

                case STAND_BY:
                default:
                    break;
            }

            // reset & start timer
            activeTimer.reset();
            activeTimer.start();
            activeTimeMs = 0;

            // reset "Above the deploy point" counter
            aboveThePointCounter = 0;

            currentState = ACTIVE;
            return 1;
        }

        // データを読み取る
        uint8_t read()
        {
            // すでに処理が走っている場合は中止
            if (currentState == BUSY) {
                return 0;
            }

            currentState = BUSY;

            // current time (ms)
            activeTimeMs = activeTimer.read_ms();

            // ms5607
            currentPressure = ms5607->getPressure();
            currentTemperature = ms5607->getTemperature();
            currentAltitude = ms5607->getAltitude();
            if(currentAltitude > altitudeToDeploy ) {
                aboveThePointCounter++;
            }

            currentState = ACTIVE;

            return 1;
        }

        /**
         * ステータスに関係なく強制的にセンサの値を更新する
         * (速度が要求されない待機状態での使用を想定)
         */
        void forceRead()
        {
            // ms5607
            currentPressure = ms5607->getPressure();
            currentTemperature = ms5607->getTemperature();
            currentAltitude = ms5607->getAltitude();
        }

        void end()
        {
            activeTimer.stop();
            currentState = STAND_BY;
        }

        bool shouldDeployParachute()
        {
            if(aboveThePointCounter < POINT_THRESHOLD) {
                return false;
            }

            return currentAltitude <= altitudeToDeploy;
        }

        SensorState getCurrentState()
        {
            return currentState;
        }

    protected:

    private:
        SensorState currentState;
        Timer activeTimer;

    };

}

#endif //SPC2019_PARACHUTE_MBED_SENSORMANAGER_H
