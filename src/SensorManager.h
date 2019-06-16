//
// Created by iwait on 6/7/19.
//

#ifndef SPC2019_PARACHUTE_MBED_SENSORMANAGER_H
#define SPC2019_PARACHUTE_MBED_SENSORMANAGER_H

#include <mbed.h>
#include <MS5607I2C.h>

#define POINT_THRESHOLD 3
#define DEEM_FLYING_AT      35.0f   //飛行中とみなす高度(m)（地表高度に加算）
#define DEPLOY_PARACHUTE_AT 30.0f   //パラシュート開放高度(m)（地表高度に加算）

namespace greysound {

    class SensorManager {

    public:

        MS5607I2C *ms5607;

        int activeTimeMs;
        float currentPressure;      // 現在の気圧
        float currentAltitude;      // 現在の高度
        float currentTemperature;   // 現在の気温
        float groundAltitude;       // 地表高度
        float flyingAltitude;       // 飛行中とみなす高度（＝飛行高度）
        float maxAltitude;          // 最大到達高度
        float deployAltitude;       // パラシュート開放高度
        uint8_t flyingAltitudeCounter; // 飛行高度到達カウンタ
        uint8_t fallingAltitudeCounter; // 落下検知カウンタ
        uint8_t deployAltitudeCounter; // パラシュート開放高度到達カウンタ

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
        SensorManager(PinName _sda, PinName _scl, uint8_t _agAddr, uint8_t _mAddr)
        {
            // インスタンス初期化
            ms5607  = new MS5607I2C(_sda, _scl, true); // sda, scl csb(1:0xEC 0:0xEE)
            activeTimer = new Timer();

            // 変数初期化
            currentPressure = 0.0f;
            currentAltitude = 0.0f;
            currentTemperature = 0.0f;
            groundAltitude = 0.0f;
            flyingAltitude = 0.0f;
            maxAltitude    = 0.0f;
            deployAltitude = 0.0f;

            // カウンタリセット
            //resetCounters();
            activeTimeMs = 0;
            flyingAltitudeCounter  = 0;
            fallingAltitudeCounter = 0;
            deployAltitudeCounter  = 0;

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
            delete ms5607;
            delete activeTimer;
        }

        /**
         * データ取得のための準備を行う
         * //FIXME: 飛行中の電源喪失を想定して書き直す(設定値をSRAMから読み出す)
         */
        uint8_t getStandBy()
        {
            uint8_t result;

            // update ground altitude
            result = updateAltitudesFromCurrentAltitude();

            // error
            if(result != 0) {
                return 1;
            }

            // カウンタリセット
            resetCounters();

            // スタンバイ状態に遷移
            currentState = STAND_BY;

            return 0;
        }

        /**
         * 現在の高度を地表高度として設定し、それを基に上空高度と開放高度を設定する
         * @return
         */
        uint8_t updateAltitudesFromCurrentAltitude()
        {
            const uint8_t numberOfSamples = 16;
            float groundAltitudeSamples = 0.0f;

            //TODO: 以下の処理で問題が発生した場合は 1 を返す

            // 現在高度をサンプリングする
            //FIXME: サンプリングせずにSRAM経由で即設定するロジックを追加する
            for (uint8_t i = 0; i<numberOfSamples; i++) {
                groundAltitudeSamples += ms5607->getAltitude();
            }

            // 現在高度の平均値を地表高度として設定する
            groundAltitude = (groundAltitudeSamples / numberOfSamples);

            // 飛行中とみなす高度を設定する
            flyingAltitude = groundAltitude + DEEM_FLYING_AT; //TODO: DEEM_FLYING_AT を廃止してSRAMを使用する

            // パラシュート開放高度を設定する
            deployAltitude = groundAltitude + DEPLOY_PARACHUTE_AT; //TODO: DEPLOY_PARACHUTE_AT を廃止してSRAMを使用する

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

                case ACTIVE:
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
            currentAltitude = ms5607->getAltitude();

            // 現在高度が最大到達高度を上回っていたら更新
            if (currentAltitude > maxAltitude) {
                maxAltitude = currentAltitude;
            }

            // 飛行状態チェック：現在高度が飛行高度を上回っている場合はカウントアップ
            if(currentAltitude > flyingAltitude && flyingAltitudeCounter < POINT_THRESHOLD) {
                flyingAltitudeCounter++;
            }

            // 落下状態チェック：最大到達高度より 2.0m 以上降下していたらカウントアップ
            if((currentAltitude + 2.0f) < maxAltitude &&  fallingAltitudeCounter < POINT_THRESHOLD) {
                fallingAltitudeCounter++;
            }

            // パラシュート開放チェック：飛行状態で、現在高度がパラシュート開放高度を下回った場合はカウントアップ
            if(isFalling() && currentAltitude < deployAltitude && deployAltitudeCounter < POINT_THRESHOLD) {
                deployAltitudeCounter++;
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
            currentAltitude = ms5607->getAltitude();
            currentTemperature = ms5607->getTemperature();
        }

        void end()
        {
            activeTimer->stop();
            currentState = STAND_BY;
        }

        /**
         * 飛行状態か？（＝飛行高度に達しているか？）
         * @return
         */
        bool isFlying()
        {
            return (flyingAltitudeCounter >= POINT_THRESHOLD);
        }

        /**
         * 落下状態か？（最大到達高度を下回っているか？）
         * @return
         */
        bool isFalling()
        {
            return (fallingAltitudeCounter >= POINT_THRESHOLD);
        }

        /**
         * パラシュートを開放してもよい状態か？
         * @return
         */
        bool isOkToDeployParachute()
        {
            return (deployAltitudeCounter >= POINT_THRESHOLD);
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

        /**
         * カウンタ群リセット
         */
        void resetCounters() {
            activeTimeMs = 0;
            flyingAltitudeCounter  = 0;
            fallingAltitudeCounter = 0;
            deployAltitudeCounter  = 0;
        }

    };

}

#endif //SPC2019_PARACHUTE_MBED_SENSORMANAGER_H
