#include "mbed.h"
 
/* ピンの機能設定  */
Serial PC (USBTX,USBRX);            //USB :シリアル通信
DigitalOut USSTriger (p11);         //P11 :超音波センサ トリガ出力
Timer ActiveTime;
 
/* 割り込み処理宣言 */
Ticker TrigerTiming;                //Trigerピン :インターバルタイマ
InterruptIn USSEcho (p12);          //p12 :超音波センサ  エコー入力
 
/* 関数宣言 */
void init(void);
void Output_Monitor(unsigned short Value);
 
/* グローバル変数宣言*/
unsigned short USSDistance;         //USSDistance:超音波センサ測定距離
 
/* main関数開始*/
int main() {
    char val;                       //val:PC.readable初期化用変数
    init();
    while(1) {
      PC.printf("%hu\n", USSDistance);
    }
}
 
 
/***************************************************
 * @brief       60ms毎の割り込みでUSSTrigerに10usのON出力
 * @param       なし
 * @return      なし
 * @date 2014/12/16 新規作成
 **************************************************/
void Triger (){
    USSTriger = 1;
    wait_us(10);
    USSTriger = 0;
}
 
/***************************************************
 * @brief       USSEcho立ち上がりでの割り込み
 * @brief       Hiの場合ActiveTimeタイマスタート
 * @param       なし
 * @return      なし
 * @date 2014/12/16 新規作成
 **************************************************/
void RiseEcho(){
    ActiveTime.start();
}
 
/***************************************************
 * @brief       USSEcho立ち下がりでの割り込み
 * @brief       Lowの場合ActiveTimeタイマ停止+値読み取り
 * @param       なし
 * @return      なし
 * @date 2014/12/16 新規作成
 **************************************************/
void FallEcho(){
    unsigned long ActiveWidth;
    ActiveTime.stop();
    ActiveWidth = ActiveTime.read_us();
    USSDistance = ActiveWidth * 0.0170;
    ActiveTime.reset();
}
 
/***************************************************
 * @brief       各種機能のプロパティ設定
 * @param       なし
 * @return      なし
 * @date 2014/12/13 新規作成
 **************************************************/
void init(void){   
    TrigerTiming.attach( Triger , 0.060 );      //USSTriger周期 60ms
    USSEcho.rise( RiseEcho );                   //USSEcho立ち上がり時割り込み
    USSEcho.fall( FallEcho );                   //USSEcho立ち下がり時割り込み
}
 
/***************************************************
 * @brief       Parameterの値をPC画面に出力
 * @param       Value : 画面に出力する値
 * @return      なし
 * @date 2014/12/14 新規作成
 **************************************************/
void Output_Monitor(unsigned short Value){
    PC.printf("%d[cm]\r\n",Value);
}