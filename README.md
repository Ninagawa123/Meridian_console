# Meridian_console
Meridianの受信配列を表示したり、Meridianのコマンドを送信するPythonです。
Dearpyguiを使用しており、ROS1の入出力に対応しています。
また、ROS信号あり, ROS信号なしが選べるので, ROSを持たないWinやMacでも手軽にMeridianの通信をテストすることができます.  
Win, Mac, Ubuntuで動作します。　　

# 準備  
$ pip3 install dearpygui など, 必要なライブラリをインストールしてください.  
ROS1への入出力を使用する場合は、49行目付近のROS用ライブラリのインポートをアンコメントアウトで不使用にしてください.  
また, 55行目付近にPCとESP32のIPアドレスを設定する箇所があるので記入してください.  
UDP_RESV_IP="192.168.1.xx" # このPCのIPアドレス  
UDP_SEND_IP="192.168.1.xx" # 送信先のESP32のIPアドレス  

# 実行  
$ CD ~/(Meridian_console.pyのあるディレクトリ)  
$ python Meridian_console.py  

で実行し、画面が表示されれば成功です. Meridianボードを立ち上げると数秒で接続が確立し, 画面の数字がチラチラと動き始めます.
標準設定の通信速度はTeensy4.0側で決定され、デフォルトでは100Hz, 1秒間に100回の往復通信を行います.

MeridianBoardの電源を入れ接続が確立すると, Meridian consoleの画面のデータがチラチラと動きます.
起動時はロボットのサーボは脱力しており、サーボの角度を手で動かすとスライダーに反映されます.

# 終了方法  
ターミナルでctrl + c を押して終了します.  

# 画面の解説  
![console_img_20230305](https://user-images.githubusercontent.com/8329123/222967600-a5a41b8b-f766-4bab-b59d-ff160feb8fbd.JPG)

#### Command
コマンドの切り替えを行います.  
Power : サーボの電源をオンにします.  
ESP32-> : ESP32からの入力データをconsole内部配列に受け取ります.  
ESP32<- : コマンドを含んだconsole内部配列のデータをESP32に出力します.  
ROS1->  : console内部配列の情報をROS1にトピックとしてパブリッシュします.  
ROS1<-  : ROS1のトピックをサブスクライブし,console内部配列に受け取ります.  
Demo<-  : consoleが生成したdemoデータをconsole内部配列に書き込みます.  
※ ROS1ボタンはROSのない環境で実行するとエラーとなります.  

デモの実行例  
「->ROS1」「->ESP32」のチェックをオフ, 「Power」「ESP32<-」「<-DEMO」のチェックをオンにすることで,   
ロボットのサーボにパワーが入り, サインカーブで構成されたダンスのデモを行います.  
Control Pad Monitor : ESP32やTeensyに接続されたリモコンの値を表示します.  
  
#### Axis Monitor  
受信したサーボの角度を表示します.  
Powerオンの時, スライダーでサーボを動かすことができます.  
  
#### Sensor Monitor  
6軸センサーや9軸センサーの値を表示します.  
SetYawボタンでヨー軸の中央値をリセットします.  

#### Button Input  
リモコンのボタン入力値をPC起点で出力します.  
  
#### Mini Terminal (2023.03.5)  
Meridim配列に差し込みたいデータ値を8つまで同時送信することができます.  
MeridianのIndex（Meridim90であれば0~89）とdataをテキストボックスに入力後, Setボタンを押し ,さらにSendにチェックを入れることでデータが送信されます.  
※ Setを行うことでバッファにデータがセットされ, Sendのチェックを入れることでそのデータが送信されつづけます.  
※ Indexの範囲外のデータは無効となり送信されません. また, チェックを外した時に送信バッファの各Indexに-1が代入されます  
  
#### Message  
IPと各経路のエラーカウント, エラー率, フレーム数, 動作周波数を表示します.  
ResetCounter: カウンタの値をリセットするボタンです.  
TsySKIP, PcSKIP: 連番データの取りこぼし数を表示します.  
※ PS4リモコン接続時に受信スキップ回数が5%ほど検出されるのは,現在の仕様では正常な動作です.  
