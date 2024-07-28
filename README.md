# [Meridian_console](https://github.com/Ninagawa123/Meridian_console) 
Meridianの受信配列を表示したり、Meridianのコマンドを送信するPythonです.  
Dearpyguiを使用しており、ROS1の入出力に対応しています.  
また、ROS信号あり, ROS信号なしが選べるので, ROSを持たないWinやMacでも手軽にMeridianの通信をテストすることができます.  
Win, Mac, Ubuntuで動作します.  
  
## 準備  
$ pip3 install dearpygui など, 必要なライブラリをインストールしてください.  

## 実行  
$ CD ~/(Meridian_console.pyのあるディレクトリ)  
$ python Meridian_console.py [送信先のESP32のIPアドレス　例:192.168.1.12]  
(送信先のIPアドレスを記入しない場合、pythonコード内のUDP_SEND_IP_DEFで設定された値で接続を試みます)  
  
で実行し、画面が表示されれば成功です. Meridianボードを立ち上げると数秒で接続が確立し, 画面の数字がチラチラと動き始めます.  
標準設定の通信速度はTeensy4.0もしくはESP32側で決定され、デフォルトでは100Hz, 1秒間に100回の往復通信を行います.  
  
起動時はロボットのサーボは脱力しており、サーボの角度を手で動かすとスライダーに反映されます.  
  
## 終了方法  
ターミナルでctrl + c を押して終了します.  
  
## 画面の解説  
<img width="851" alt="console20231230" src="https://github.com/Ninagawa123/Meridian_console/assets/8329123/0d2df355-8369-44ce-b4e0-bdbb73569354">
  
#### Axis Monitor  
受信したサーボの角度を表示します.  
Powerオンの時, スライダーでサーボを動かすことができます. (CommandウィンドウのPower,Python,Enebleの３箇所にチェックを入れて下さい)   
Home   : すべてのサーボポジションに対し一度0を代入します.  
Target : ボードへの送信データを表示します.  
Actual : ボードからの受信データを表示します（サーボの動作結果位置など）  
  
#### Sensor Monitor  
6軸センサーや9軸センサーの値を表示します.  
SetYaw : ヨー軸の中央値をリセットします.  
  
#### Command  
POWER  : 全サーボのパワーをオンオフします.  
DEMO   : サインカーブの全身モーションを計算します.  
Python : ユーザー定義のpythonコードを反映します.  
Enable : Demoやpythonを送信に反映します.  
->ROS1 : ROS1のjointデータをパブリッシュします.（Rvisと連動できます）  
<-ROS1 : ROS1のサブスクライブします.(動作未確認)  
Control Pad Monitor: ESP32やTeensyに接続されたリモコンの値を標準化して表示します.  
  
デモの実行例  
「DEMO」「Power」「Enable」のチェックをオンにすることで,  
ロボットのサーボにパワーが入り, サインカーブで構成されたダンスのデモを行います.  
  
#### Button Input  
リモコンのボタン入力値をPC起点で出力します.  
  
#### Mini Terminal  
Meridim配列に差し込みたいデータ値を8つまで同時送信することができます.  
MeridianのIndex（Meridim90であれば0~89）とdataをテキストボックスに入力後, Setボタンを押し,　さらにSendにチェックを入れることでデータが送信されます.  
※ Setを行うことでバッファにデータがセットされ, Sendのチェックを入れることでそのデータが送信されつづけます.  
※ Indexの範囲外のデータは無効となり送信されません. また, チェックを外した時に送信バッファの各Indexに-1が代入されます.  
Flow/Step  : フローモードとステップモードを切り替えます.  
Next frame : ステップモード時に1フレームだけ次に進みます.  
  
#### Message  
IPと各経路のエラーカウント, エラー率, フレーム数, 動作周波数を表示します.  
ResetCounter: カウンタの値をリセットするボタンです.  
TsySKIP, PcSKIP: 連番データの取りこぼし数を表示します.  
※ ESP32にPS4リモコンを接続した際に受信スキップ回数が5%ほど検出されるのは,現在の仕様では正常な動作です.  
※ I2C接続のMerimoteを使用した場合、PS4リモコン接続での受信スキップ回数はほぼ0になります。

#### バージョンv0.0.2
リセットボタンやリモコンアナログ入力値表示など細かな修正をしました。
