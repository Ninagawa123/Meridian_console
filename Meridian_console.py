# !/usr/bin/python3
# coding: UTF-8
# もしくは　# !/usr/bin/env python など環境に合わせて

# Izumi Ninagawa & Meridian project
# MIT License

# 2023.12.27 これまでのグローバル変数をクラスに格納. 一部をnumpy化. sleepを入れてCPU負荷を軽減

# Meridian console 取扱説明書
#
# ・起動方法
# 当ファイルがあるディレクトリにて, ターミナルより
# python3 Meridian_console.py [送信先のESP32のIPアドレス　例:192.168.1.12]
# と入力して実行します. 必要に応じてライブラリをpip3で追加してください
# （IPアドレスなしで実行した場合は、82行目のUDP_SEND_IP_DEFでの設定が反映されます。）
# UDP_SEND_IPはESP32の起動時にPCシリアルモニタ上に表示されます
#
# ・各ウィンドウについて
# 【Axis Monitor】
# 各サーボの値です. パワーオン時にはスライダでサーボを動かすことができます.
#
# 【Sensor Monitor】
# MIUのデータを表示します. rol,pit,yawはセンサフュージョン値です. SetYawボタンでヨー軸の中央値をリセットできます
#
# 【Command】
# POWER: 全サーボのパワーをオンオフします
# ->ESP32: ESP32から受信したデータを外部出力用として有効にします
# <-ESP32: 外部入力データおよびコンソール計算結果のESP32への送信を有効にします
# ->ROS1: ROS1のjointデータをパブリッシュします（Rvisと連動できます）
# <-ROS1: ROS1のサブスクライブします（動作未確認）
# <-DEMO: サインカーブの全身モーションを計算します（<-ESP32オンで送信）
# Control Pad Monitor: リモコンの入力状態を標準化して表示します
#
# 【Mini Terminal】
# Meridim配列のデータをインプットし8つまで同時送信することができます.
# MeridianのIndex（Meridim90であれば0~89）とDataを入力し、Setボタンを押し、
# Sendにチェックを入れることでデータが送信されます。
# ※Setを行うことでバッファにデータがセットされ、Sendのチェックを入れることでそのデータが送信されつづけます。
# 　Indexの範囲外のデータは無効となり送信されません。また、チェックを外した時に送信バッファの各Indexに-1が代入されます。
#
# 【Button Input】
# コンソールからリモコンボタン押下情報を送信します
#
# 【Message】
# IPと各経路のエラーカウント、エラー率、フレーム数、動作周波数を表示します
# ResetCounter: カウンタの値をリセットするボタンです
# TsySKIP, PcSKIP: 連番データの取りこぼし数を表示します
# PS4リモコン接続時に受信スキップ回数が5%ほど検出されるのは、現在の仕様では正常な動作です
# Button Inputウィンドウ
# コンソールからリモコンボタン押下情報を送信します

import sys
# from ast import Pass
import numpy as np
import socket
from contextlib import closing
import struct
import math
import dearpygui.dearpygui as dpg
import threading
import signal
import time
import atexit
import struct

# ROS搭載マシンの場合はrospyをインポートする
try:
    import rospy
    rospy_imported = True
    from sensor_msgs.msg import JointState
except ImportError:
    rospy_imported = False
    print("rospy not found. ROS functions will be disabled.")


# 定数
TITLE_VERSION = "Meridian_Console_v23.1227"  # DPGのウィンドウタイトル兼バージョン表示
UDP_RESV_PORT = 22222                       # 受信ポート
UDP_SEND_PORT = 22224                       # 送信ポート
UDP_SEND_IP_DEF = "192.168.1.85"            # 送信先のESP32のIPアドレス 21
MSG_SIZE = 90                               # Meridim配列の長さ(デフォルトは90)
MSG_BUFF = MSG_SIZE * 2                     # Meridim配列のバイト長さ
MSG_ERRS = MSG_SIZE - 2                     # Meridim配列のエラーフラグの格納場所（配列の最後から２番目）
MSG_CKSM = MSG_SIZE - 1                     # Meridim配列のチェックサムの格納場所（配列の末尾）
STEP = 94                                   # 1フレームあたりに増加させる制御処理用の数値,サインカーブを何分割するか

# マスターコマンド
MCMD_TORQUE_ALL_OFF = 0                     # すべてのサーボトルクをオフにする（脱力）
MCMD_UPDATE_YAW_CENTER = 10002              # センサの推定ヨー軸を現在値でゼロに
MCMD_ENTER_TRIM_MODE = 10003                # トリムモードに入る(現在不使用)
MCMD_CLEAR_SERVO_ERROR_ID = 10004           # 通信エラーのサーボのIDをクリア
MCMD_BOARD_TRANSMIT_ACTIVE = 10005          # ボードが定刻で送信を行うモード（デフォルト設定.PC側が受信待ち）
MCMD_BOARD_TRANSMIT_PASSIVE = 10006         # ボードが受信を待ち返信するモード（PC側が定刻送信）


class MeridianConsole:
    def __init__(self):
        # ここにグローバル変数をクラスの属性として定義
        self.message0 = "This PC's IP adress is "+get_local_ip()
        self.message1 = ""
        self.message2 = ""
        self.message3 = ""
        self.message4 = ""
        self.flag_udp_resv = True
        self.loop_count = 0

        # Meridim配列関連
        self.r_meridim = np.zeros(
            MSG_SIZE, dtype=np.int16)           # Meridim配列
        self.s_meridim = np.zeros(
            MSG_SIZE, dtype=np.int16)           # Meridim配列
        self.r_meridim_char = np.zeros(
            MSG_SIZE*2, dtype=np.uint8)    # Meridim配列
        self.r_meridim_ushort = np.zeros(
            MSG_SIZE*2, dtype=np.uint8)  # Meridim配列

        # ROSからサブスクライブしたサーボ位置情報の格納用Meridim配列
        self.s_meridim_js_sub = [0]*MSG_SIZE
        # Meridim配列のPC側で作成したサーボ位置命令送信用
        self.s_meridim_motion = [0]*MSG_SIZE
        self.s_meridim_motion_keep = [0]*MSG_SIZE   # Meridim配列のパワーオン時の位置キープ用
        self.s_minitermnal_keep = np.zeros((8, 2))  # コンパネからのリモコン入力用

        # エラー集計表示用変数
        self.loop_count = 1  # フレーム数のカウンタ
        self.error_count_esp_to_pc = 0   # PCからESP32へのUDP送信でのエラー数
        self.error_count_pc_to_esp = 0   # ESP32からPCへのUDP送信でのエラー数
        self.error_count_esp_to_tsy = 0  # ESPからTeensyへのSPI通信でのエラー数
        self.error_count_tsy_delay = 0   # Teensyのシステムディレイ
        self.error_count_tsy_to_esp = 0  # TeensyからESP32へのSPI通信でのエラー数
        self.error_count_tsy_skip = 0    # Teensyが受信したデータがクロックカウントスキップしていたか
        self.error_count_esp_skip = 0    # ESPが受信したデータがクロックカウントスキップしていたか
        self.error_count_pc_skip = 0     # PCが受信したデータがクロックカウントスキップしていたか
        self.error_count_servo_skip = 0  # マイコン(Teensy/ESP32)がサーボ値の受信に失敗した回数
        self.error_servo_id = "None"     # 受信エラーのあったサーボのIDを格納
        self.frame_sync_s = 56000        # 送信するframe_sync_r(0-59999)
        self.frame_sync_r_expect = 0     # 毎フレームカウントし、受信カウントと比較(0-59999)
        self.frame_sync_r_resv = 0       # 今回受信したframe_sync_r
        self.frame_sync_r_last = 0       # 前回受信したframe_sync_r
        self.error_servo_id_past = 0

# 制御コマンド用フラグ等
        self.main_command = MSG_SIZE         # meridim[0]に格納するコマンド番号
        self.flag_update_yaw = 0             # IMUのヨー軸センターリセットフラグ(python内部用)
        self.flag_servo_power = 0            # 全サーボのパワーオンオフフラグ
        self.flag_udp_resv = 0               # UDP受信の完了フラグ
        # ESP32からの状態データの受信のオンオフフラグ（モーション送信時のシミュレーション空間用として）
        self.flag_resv_data = 0
        self.flag_send_data = 0              # ESP32への状態データの送信のオンオフフラグ（サーボパワーオフでもデータ送信可能にすべく）
        self.flag_send_virtual = 0           # ハードウェアを接続しないで動作させる場合のバーチャルハードのオンオフフラグ
        self.flag_send_motion = 0            # 計算モーション送信のオンオフフラグ
        self.flag_send_miniterminal_data = 0
        self.flag_set_miniterminal_data = 0  # ミニターミナルの値をセットするボタンのためのフラグ
        self.flag_tarminal_mode_send = 0     # miniterminalを有効にし、コマンドを優先する。
        self.flag_demo_action = 0            # デモ/テスト用の計算モーション送信のオンオフフラグ
        self.flag_ros1_pub = 0               # ROS1のjoint_statesのパブリッシュ
        self.flag_ros1_sub = 0               # ROS1のjoint_statesのサブスクライブ
        self.flag_ros1 = 0                   # ROS1の起動init（初回のみ）
        self.flag_set_flow = True            # Meridianの循環をTrue:通常フロー、False:ステップ
        self.flag_flow_switch = False        # フロー、ステップのモード切り替え時に反応するフラグ
        self.flag_servo_home = 0             # 全サーボ位置をゼロリセット
        self.flag_send_data_step_frame = 0   # RETURN押下で１ステップ分のデータ送信

        self.pad_button_panel_short = np.array(
            [0], dtype=np.uint16)  # コンパネからのリモコン入力用

        # メッセージ表示用
        self.message3 = ""
        self.message4 = ""

        # モーション計算用変数
        self.x = 0  # 増分計算用 (STEPずつ)
        self.y = 0  # 増分計算用 (1ずつ)

        self.jspn = list(range(30))  # サーボ角度のROS joint_states変換用の回転方向順逆補正
        self.jspn[0] = 1   # 頭ヨー
        self.jspn[1] = 1   # 左肩ピッチ
        self.jspn[2] = 1   # 左肩ロール
        self.jspn[3] = 1   # 左肘ヨー
        self.jspn[4] = 1   # 左肘ピッチ
        self.jspn[5] = 1   # 左股ヨー
        self.jspn[6] = 1   # 左股ロール
        self.jspn[7] = 1   # 左股ピッチ
        self.jspn[8] = 1   # 左膝ピッチ
        self.jspn[9] = 1   # 左足首ピッチ
        self.jspn[10] = 1  # 左足首ロール
        self.jspn[11] = 1  # 予備
        self.jspn[12] = 1  # 予備
        self.jspn[13] = 1  # 予備
        self.jspn[14] = 1  # 予備
        self.jspn[15] = 1  # 腰ヨー
        self.jspn[16] = 1  # 右肩ピッチ
        self.jspn[17] = -1  # 右肩ロール
        self.jspn[18] = -1  # 右肘ヨー
        self.jspn[19] = 1  # 右肘ピッチ
        self.jspn[20] = -1  # 右股ヨー
        self.jspn[21] = -1  # 右股ロール
        self.jspn[22] = 1  # 右股ピッチ
        self.jspn[23] = 1  # 右膝ピッチ
        self.jspn[24] = 1  # 右足首ピッチ
        self.jspn[25] = -1  # 右足首ロール
        self.jspn[26] = 1  # 予備
        self.jspn[27] = 1  # 予備
        self.jspn[28] = 1  # 予備
        self.jspn[29] = 1  # 予備
        self.start = 0

        # ロックの追加
        self.lock = threading.Lock()


def get_local_ip():
    try:
        # ネットワーク上で接続を確立するためのダミーのsocketを作成します
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # GoogleのDNSサーバーに接続を試みますが、実際には接続しません
        s.connect(("8.8.8.8", 80))
        # このsocketを通じて取得されるローカルIPアドレスを取得します
        IP = s.getsockname()[0]
        s.close()
        return IP
    except Exception as e:
        return "Error: " + str(e)


def get_udp_send_ip():
    # コマンドライン引数が提供されているか確認
    if len(sys.argv) > 1:
        return sys.argv[1]  # 最初の引数を返す
    else:
        return UDP_SEND_IP_DEF  # デフォルトのIPアドレス（またはエラーメッセージ）


# マスターコマンド用の定数(Meridim配列0番に格納する値)
CMD_SET_YAW_CENTER = 10002  # IMUのヨー軸センターリセットコマンド


# Meridanの循環をステップ（PCからの待ち受け）モードに

command_send_trial = 1  # Commandを連続で送信する回数


# Meridim配列のパワーオン時の位置キープ用

# Meridianデータのインスタンス
mrd = MeridianConsole()

################################################################################################################################
## 　データの送受信　###############################################################################################################
################################################################################################################################


def meridian_loop():
    # UDP用のsocket設定
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((get_local_ip(), UDP_RESV_PORT))
    _checksum = np.array([0], dtype=np.int16)
    atexit.register(cleanup)  # この行は機能しているかどうかわからない

    while (True):
        print("Start.")
        # 180個の要素を持つint8型のNumPy配列を作成
        _r_bin_data_past = np.zeros(180, dtype=np.int8)
        _r_bin_data = np.zeros(180, dtype=np.int8)
        mrd.message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."

        with closing(sock):
            while True:

                mrd.loop_count += 1  # このpythonを起動してからのフレーム数をカウントアップ
                _r_bin_data_past = _r_bin_data
                _r_bin_data, addr = sock.recvfrom(MSG_BUFF)  # UDPに受信したデータを転記

                ######## [1-1] UDPデータの受信 ##############################################################################################
                while np.array_equal(_r_bin_data_past, _r_bin_data):  # ここで受信データの更新を待つ
                    _r_bin_data, addr = sock.recvfrom(MSG_BUFF)      # 受信データを更新

                ######## [1-2] 受信UDPデータの変換 ###########################################################################################
                # 受信データをshort型のMeridim90に変換
                mrd.r_meridim = struct.unpack('90h', _r_bin_data)
                mrd.r_meridim_ushort = struct.unpack(
                    '90H', _r_bin_data)  # unsignedshort型
                mrd.r_meridim_char = struct.unpack('180b', _r_bin_data)
                mrd.message1 = "UDP data receiving from "+UDP_SEND_IP  # 受信中のメッセージ表示

                ######## [1-3] 受信UDPデータのチェック ########################################################################################
                # 受信データに対するチェックサム値の計算
                _checksum[0] = ~np.sum(mrd.r_meridim[:MSG_SIZE-1])

                # エラーフラグ各種のカウントアップ
                # short型のビットをpythonで扱うためのnumpy配列テンポラリ変数
                _temp_int16 = np.array([0], dtype=np.int16)

                if _checksum[0] == mrd.r_meridim[MSG_SIZE-1]:
                    # エラーフラグ14ビット目（ESP32のPCからのUDP受信のエラーフラグ）を調べる
                    if (mrd.r_meridim[MSG_ERRS] >> 14 & 1) == 1:
                        mrd.error_count_pc_to_esp += 1
                    # エラーフラグ13ビット目（TeensyのESP32からのSPI受信のエラーフラグ）を調べる
                    if (mrd.r_meridim[MSG_ERRS] >> 13 & 1) == 1:
                        mrd.error_count_esp_to_tsy += 1
                    # エラーフラグ12ビット目（ESPのTeensyからのSPI受信のエラーフラグ）を調べる
                    if (mrd.r_meridim[MSG_ERRS] >> 12 & 1) == 1:
                        mrd.error_count_tsy_to_esp += 1
                    # エラーフラグ11ビット目（Teensyのシステムディレイのエラーフラグ）を調べる
                    if (mrd.r_meridim[MSG_ERRS] >> 11 & 1) == 1:
                        mrd.error_count_tsy_delay += 1
                    # エラーフラグ10ビット目（ESPのPCからのUDP受信のフレーム連番スキップフラグ）を調べる
                    if (mrd.r_meridim[MSG_ERRS] >> 10 & 1) == 1:
                        mrd.error_count_esp_skip += 1
                    # エラーフラグ9ビット目（TeensyのESP経由のPCから受信のフレーム連番スキップフラグ）を調べる
                    if (mrd.r_meridim[MSG_ERRS] >> 9 & 1) == 1:
                        mrd.error_count_tsy_skip += 1

                    _temp_int16[0] = mrd.r_meridim[MSG_ERRS] & 0b0000000011111111
                    mrd.error_servo_id_past = mrd.error_servo_id

                    # サーボ値の受信に失敗したサーボID(エラーフラグ下位8ビット）を調べる
                    if _temp_int16[0] > 0:
                        mrd.error_count_servo_skip += 1    #
                        if mrd.r_meridim[MSG_ERRS] & 0b0000000011111111 > 99:
                            mrd.error_servo_id = "id_R" + \
                                str(int(mrd.r_meridim[MSG_ERRS]
                                    & 0b0000000011111111)-100)
                        else:
                            mrd.error_servo_id = "id_L" + \
                                str(int(
                                    mrd.r_meridim[MSG_ERRS] & 0b0000000011111111))
                    else:
                        mrd.error_servo_id = "None"

                    if (mrd.error_servo_id_past != mrd.error_servo_id):
                        if mrd.error_servo_id == "None":
                            print("Servo error gone...")
                        else:
                            print("Found servo error: "+mrd.error_servo_id)

                    # エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を下げる
                    _temp_int16[0] = mrd.r_meridim[MSG_ERRS] & 0b0111111111111111
                else:
                    # エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を上げる
                    _temp_int16[0] = mrd.r_meridim[MSG_ERRS] | 0b1000000000000000
                    mrd.error_count_esp_to_pc += 1  # PCのUDP受信エラーをカウントアップ

                # フレームスキップチェック用のカウントの受信と処理
                mrd.frame_sync_r_resv = mrd.r_meridim_ushort[1]  # 受信カウントを代入

#                if mrd.flag_set_flow == False:
#                    while(mrd.flag_send_data_step_frame == False):
#                        time.sleep(0.005)  # CPUの負荷を下げる
#                    mrd.flag_send_data_step_frame == False

                ######## [2-1] チェック済み受信UDPデータに基づく処理 ##################################################################
                # チェックサムがOK かつ シーケンス番号が前回と異なっていれば、処理に回す
                if (_checksum[0] == mrd.r_meridim[MSG_SIZE-1]) and (mrd.frame_sync_r_resv != mrd.frame_sync_r_last):

                    # 受信データを送信データに転記
                    for i in range(MSG_SIZE-1):
                        mrd.s_meridim[i] = mrd.r_meridim[i]

                    # 受信予測用のカウントアップ
                    mrd.frame_sync_r_expect += 1
                    if mrd.frame_sync_r_expect > 59999:
                        mrd.frame_sync_r_expect = 0

                    if (mrd.frame_sync_r_resv == mrd.frame_sync_r_expect):  # 受信したカウントが予想通りであればスキップなし
                        # PCのESP経由Teensyからの連番スキップフラグを下げる
                        _temp_int16[0] &= 0b1111111011111111
                    else:
                        # PCのESP経由Teensyからの連番スキップフラグを上げる
                        _temp_int16[0] |= 0b0000000100000000
                        mrd.frame_sync_r_expect = mrd.frame_sync_r_resv   # 受信カウントの方が多ければズレを検出し、追いつく
                        mrd.error_count_pc_skip += 1  # スキップカウントをプラス

                    # PC側サーボ位置発信用に最終サーボ情報をキープ
                    if mrd.flag_servo_power == 2:  # サーボオンボタン押下初回のみ最終受け取りサーボ情報をキープ
                        for i in range(21, 81, 2):
                            mrd.s_meridim_motion[i] = mrd.r_meridim[i]
                            mrd.s_meridim_motion_keep[i] = mrd.r_meridim[i]
                        mrd.flag_servo_power = 1

                ######## [3-1] 送信用UDPデータの作成 ############################################
                    # 送信用のモーションを作成（①受信値そのまま ②ROSサブスク反映 ③計算モーション）
                    if _checksum[0] == mrd.r_meridim[MSG_SIZE-1]:  # 受信成功時はデータ更新
                        mrd.s_meridim = []  # データのクリア
                        mrd.s_meridim = list(mrd.r_meridim)

                    # ①受信値そのままの場合：送信データのベースを受信データのコピーで作成
                    if mrd.flag_servo_power:  # サーボパワーオン時は、電源入力時に保持した値を固定で流す（ハウリング的なサーボ位置ズレの増幅を防止）
                        for i in range(21, 81, 2):
                            mrd.s_meridim[i] = mrd.s_meridim_motion_keep[i]
                    else:
                        if mrd.flag_resv_data:
                            for i in range(21, 81, 2):  # 受信サーボ値を書き込みモーションのベースとして一旦キープ
                                mrd.s_meridim_motion[i] = mrd.r_meridim[i]

                    # ②サーボ位置にROSのサブスクライブを反映させる場合にはここでデータを作成★★
                    if mrd.flag_ros1_sub:
                        for i in range(15):
                            mrd.s_meridim_motion[21+i *
                                                 2] = mrd.s_meridim_js_sub[21+i*2]
                            mrd.s_meridim_motion[51+i *
                                                 2] = mrd.s_meridim_js_sub[51+i*2]

                    # ③サーボ位置をここで計算制御する場合は以下でデータを作成(まずはデモモーションのみで運用テスト)
                    if mrd.flag_demo_action:
                        # xをフレームごとにカウントアップ
                        mrd.x += math.pi/STEP
                        if mrd.x > math.pi*2000:
                            mrd.x = 0
                        # サインカーブで全身をくねらせる様にダンス
                        mrd.s_meridim_motion[21] = int(
                            np.sin(mrd.x)*3000)           # 頭ヨー
                        mrd.s_meridim_motion[23] = int(
                            np.sin(mrd.x)*1000) + 2000    # 左肩ピッチ
                        mrd.s_meridim_motion[25] = - \
                            int(np.sin(mrd.x*2)*1000) + 1000  # 左肩ロール
                        mrd.s_meridim_motion[27] = int(
                            np.sin(mrd.x)*1000) + 1000    # 左肘ヨー
                        mrd.s_meridim_motion[29] = int(
                            np.sin(mrd.x)*3000) - 3000    # 左肘ピッチ
                        mrd.s_meridim_motion[31] = int(
                            np.sin(mrd.x)*500)            # 左股ヨー
                        mrd.s_meridim_motion[33] = - \
                            int(np.sin(mrd.x)*400)           # 左股ロール
                        mrd.s_meridim_motion[35] = int(
                            np.sin(mrd.x*2)*2000) - 200   # 左股ピッチ
                        mrd.s_meridim_motion[37] = - \
                            int(np.sin(mrd.x*2)*4000)        # 左膝ピッチ
                        mrd.s_meridim_motion[39] = int(
                            np.sin(mrd.x*2)*2000) - 200   # 左足首ピッチ
                        mrd.s_meridim_motion[41] = int(
                            np.sin(mrd.x)*400)            # 左足首ロール
                        mrd.s_meridim_motion[51] = - \
                            int(np.sin(mrd.x)*2000)          # 腰ヨー
                        mrd.s_meridim_motion[53] = - \
                            int(np.sin(mrd.x)*1000) + 2000   # 右肩ピッチ
                        mrd.s_meridim_motion[55] = - \
                            int(np.sin(mrd.x*2)*1000) + 1000  # 右肩ロール
                        mrd.s_meridim_motion[57] = - \
                            int(np.sin(mrd.x)*1000) + 1000   # 右肘ヨー
                        mrd.s_meridim_motion[59] = - \
                            int(np.sin(mrd.x)*3000) - 3000   # 右肘ピッチ
                        mrd.s_meridim_motion[61] = - \
                            int(np.sin(mrd.x)*500)           # 右股ヨー
                        mrd.s_meridim_motion[63] = int(
                            np.sin(mrd.x)*400)            # 右股ロール
                        mrd.s_meridim_motion[65] = - \
                            int(np.sin(mrd.x*2)*2000) - 200  # 右股ピッチ
                        mrd.s_meridim_motion[67] = int(
                            np.sin(mrd.x*2)*4000)         # 右膝ピッチ
                        mrd.s_meridim_motion[69] = - \
                            int(np.sin(mrd.x*2)*2000) - 200  # 右足首ピッチ
                        # 右足首ロール
                        mrd.s_meridim_motion[71] = -int(np.sin(mrd.x)*400)

                    # ④サーボ位置リセットボタン(Home)が押下されていたらいったん全サーボ位置を0にする
                    if mrd.flag_servo_home > 0:
                        for i in range(15):
                            mrd.s_meridim_motion[21+i*2] = 0
                            mrd.s_meridim_motion[51+i*2] = 0

                    # データを送信Meridim配列に格納

                    # コマンド値の生成と格納

                    # 送信マスターコマンドフラグチェック：ヨー軸センターリセットコマンドを格納
                    if (mrd.flag_update_yaw > 0):
                        mrd.flag_update_yaw -= 1
                        mrd.s_meridim[0] = CMD_SET_YAW_CENTER
                        if (mrd.flag_update_yaw == 0):
                            print(
                                "Send COMMAND 'Set Yaw Center.':["+str(CMD_SET_YAW_CENTER)+"]")

#                    print("flag_set_flow: "+str(mrd.flag_set_flow))

#                    if mrd.flag_set_flow == True:  # フローモード(ボード側が周期制御を持つ)への切り替え
#                        if mrd.flag_flow_switch == True:
#                            mrd.s_meridim[0] = MCMD_BOARD_TRANSMIT_ACTIVE
#                            mrd.flag_flow_switch = False

#                        dpg.set_value("transaction_mode", "Flow")

#                    if mrd.flag_set_flow == False:  # ステップモード(PC側が周期制御を持つ)への切り替え
#                        if mrd.flag_flow_switch == True:
#                            mrd.s_meridim[0] = MCMD_BOARD_TRANSMIT_PASSIVE
#                            mrd.flag_flow_switch = False
#                        #mrd.flag_set_flow = False
#                        dpg.set_value("transaction_mode", "Step")

                    else:
                        mrd.s_meridim[0] = MSG_SIZE  # デフォルト値を格納

                    # PC側発行のサーボ位置を格納
                    if mrd.flag_send_data:
                        for i in range(21, 81, 2):
                            mrd.s_meridim[i] = mrd.s_meridim_motion[i]

                    # サーボオンオフフラグチェック：サーボオンフラグを格納
                    if mrd.flag_servo_power > 0:
                        for i in range(20, 80, 2):
                            mrd.s_meridim[i] = 1
                    else:
                        for i in range(20, 80, 2):
                            mrd.s_meridim[i] = 0

                    # 送信用シーケンス番号の作成と格納
                    mrd.frame_sync_s += 1  # 送信用のframe_sync_sをカウントアップ
                    if mrd.frame_sync_s > 59999:  # 60,000以上ならゼロリセット
                        mrd.frame_sync_s = 0
                    if mrd.frame_sync_s > 32767:  # unsigned short として取り出せるようなsinged shortに変換
                        mrd.s_meridim[1] = mrd.frame_sync_s-65536
                    else:
                        mrd.s_meridim[1] = mrd.frame_sync_s  # & 0xffff

                    # リモコンデータをリセットし、PCからのリモコン入力値を格納
                    temp = np.array([0], dtype=np.int16)
                    temp[0] = 0
                    temp[0] = mrd.pad_button_panel_short[0]  # ボタンのショート型変換
                    mrd.s_meridim[15] = temp[0]  # ボタン
                    mrd.s_meridim[16] = 0  # アナログ1
                    mrd.s_meridim[17] = 0  # アナログ2
                    mrd.s_meridim[18] = 0  # アナログ3

                    # キープしたエラーフラグを格納
                    mrd.s_meridim[MSG_ERRS] = _temp_int16[0]

                    # miniterminalがモードオンならセットされた送信データを送信
                    if mrd.flag_tarminal_mode_send > 0:  # ミニターミナルからの入力データをセットする
                        print_string = ""
                        for i in range(8):
                            if ((mrd.s_minitermnal_keep[i][0] >= 0) and (mrd.s_minitermnal_keep[i][0] < MSG_SIZE)):
                                mrd.s_meridim[int(mrd.s_minitermnal_keep[i][0])] = int(
                                    mrd.s_minitermnal_keep[i][1])
                                print_string = print_string + \
                                    "["+str(int(mrd.s_minitermnal_keep[i][0]))+"] " + \
                                    str(int(mrd.s_minitermnal_keep[i][1]))+", "
                                # サーボパワーオン時のキープ配列にも反映しておく。こうするとミニターミナルから脱力してサーボを回転させた後にサーボパワーオンで位置の固定ができる。
                                mrd.s_meridim_motion_keep[int(mrd.s_minitermnal_keep[i][0])] = int(
                                    mrd.s_minitermnal_keep[i][1])

                        if mrd.flag_tarminal_mode_send == 2:  # 送信データを一回表示
                            print("Sending data : ")
                            print(print_string[:-2])  # 末尾のカンマ以外を表示
                            mrd.flag_tarminal_mode_send = 1

                    # 格納した送信データについてチェックサムを追加
                    _checksum[0] = ~np.sum(mrd.s_meridim[:MSG_SIZE-1])
                    mrd.s_meridim[MSG_SIZE-1] = _checksum[0]

                ######## [4-1] UDPデータを送信 ####################################################################################
                    s_bin_data = struct.pack(
                        '90h', *mrd.s_meridim)       # データをパック
                    sock.sendto(s_bin_data, (UDP_SEND_IP,
                                UDP_SEND_PORT))  # UDP送信
                    now = time.time()-mrd.start+0.0001

                ######## [5-1] 表示処理 ##########################################################################################
                    mrd.message2 = "ERROR COUNT ESP-PC:"+str("{:}".format(mrd.error_count_esp_to_pc)) + " PC-ESP:"+str("{:}".format(mrd.error_count_pc_to_esp))+" ESP-TSY:"+str(
                        "{:}".format(mrd.error_count_esp_to_tsy)) + " TSY_Delay:"+str("{:}".format(mrd.error_count_tsy_delay)) + "    Servo_trouble:"+mrd.error_servo_id

                    mrd.message3 = "ERROR RATE ESP-PC:"+str("{:.2%}".format(mrd.error_count_esp_to_pc/mrd.loop_count)) + " PC-ESP:"+str("{:.2%}".format(mrd.error_count_pc_to_esp/mrd.loop_count))+" ESP-TSY:"+str("{:.2%}".format(
                        mrd.error_count_esp_to_tsy/mrd.loop_count)) + " TsySKIP:"+str("{:.2%}".format(mrd.error_count_tsy_skip/mrd.loop_count)) + " ESPSKIP:" + str("{:.2%}".format(mrd.error_count_esp_skip/mrd.loop_count))

                    mrd.message4 = "SKIP COUNT Tsy:" + str("{:}".format(mrd.error_count_tsy_skip))+" ESP:"+str("{:}".format(mrd.error_count_esp_skip))+" PC:"+str("{:}".format(mrd.error_count_pc_skip)) + " Servo:"+str(
                        "{:}".format(mrd.error_count_servo_skip))+" PCframe:"+str(mrd.loop_count)+" BOARDframe:"+str(mrd.frame_sync_r_resv)+" "+str(int(mrd.loop_count/now))+"Hz"

                    # 今回受信のシーケンス番号を次回比較用にキープ
                    mrd.frame_sync_r_last = mrd.frame_sync_r_resv

                time.sleep(0.002)  # CPUの負荷を下げる


################################################################################################################################
## 　関数各種　####################################################################################################################
################################################################################################################################

def cleanup():  # ctrl+cで終了したときにも確実にソケットを閉じる試み（いまのところ機能していないかも）
    print("Meridan_console quited.")

# チェックボックスに従いサーボパワーオンフラグをオンオフ


def pad_btn_panel_on(sender, app_data, user_data):
    mrd.pad_button_panel_short
    if (mrd.pad_button_panel_short[0] & user_data) == 0:
        mrd.pad_button_panel_short[0] = mrd.pad_button_panel_short[0] | user_data
        print(f'Btn:{mrd.pad_button_panel_short[0]}')
    else:
        mrd.pad_button_panel_short[0] = mrd.pad_button_panel_short[0] ^ user_data
        print(f'Btn:{mrd.pad_button_panel_short[0]}')


def set_servo_power():  # チェックボックスに従いサーボパワーオンフラグをオンオフ
    mrd.flag_servo_power
    if mrd.flag_servo_power == 0:
        mrd.flag_servo_power = 2
        print("Servo Power ON")
    else:
        mrd.flag_servo_power = 0
        print("Servo Power OFF")


def set_demo_action():  # チェックボックスに従いアクション送信フラグをオンオフ
    if mrd.flag_demo_action == 0:
        mrd.flag_demo_action = 1
        print("Start DEMO motion data streaming.")
    else:
        mrd.flag_demo_action = 0
        print("Quit DEMO motion data streaming.")


def set_resv_data():  # チェックボックスに従いデータ送信フラグをオンオフ
    if mrd.flag_resv_data == 0:
        mrd.flag_resv_data = 1
        print("Start receiving data from ESP32.")
    else:
        mrd.flag_resv_data = 0
        print("Quit receiving data from ESP32.")


def set_send_data():  # チェックボックスに従いデータ送信フラグをオンオフ
    if mrd.flag_send_data == 0:
        mrd.flag_send_data = 1
        print("Start sending data to ESP32.")
    else:
        mrd.flag_send_data = 0
        print("Quit sending data to ESP32.")


def send_miniterminal_data():  # チェックボックスに従いデータ送信フラグをオンオフ
    if mrd.flag_send_miniterminal_data == 0:
        mrd.flag_send_miniterminal_data = 1
        print("Set mini tarminal data to ESP32.")


def set_miniterminal_data():  # ミニターミナルのセットボタンが押下されたら送信データをセットする
    print_string = ""
    for i in range(8):
        _value_tag_index = "s_index"+str(i)
        _value_tag_data = "s_data"+str(i)

        if dpg.get_value(_value_tag_index) != "":
            if dpg.get_value(_value_tag_data) != "":
                if (int(dpg.get_value(_value_tag_data)) >= -32768) and (int(dpg.get_value(_value_tag_data)) <= 32767) and (int(dpg.get_value(_value_tag_index)) >= 0) and (int(dpg.get_value(_value_tag_index)) < MSG_SIZE):
                    print_string = print_string + \
                        "[" + str(dpg.get_value(_value_tag_index)) + "] " + \
                        str(dpg.get_value(_value_tag_data)) + ", "
                    mrd.s_minitermnal_keep[i][0] = int(
                        dpg.get_value(_value_tag_index))
                    mrd.s_minitermnal_keep[i][1] = int(
                        dpg.get_value(_value_tag_data))
                else:
                    # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
                    mrd.s_minitermnal_keep[i][0] = -1
                    print_string = print_string + \
                        "["+str(dpg.get_value(_value_tag_index)) + \
                        "] out of range, "

    print("Set mini tarminal data : ")
    print(print_string[:-2])  # 末尾のカンマ以外を表示


def set_tarminal_send_on():  # ボタン押下でset_flowフラグをオン
    if mrd.flag_tarminal_mode_send == 0:
        mrd.flag_tarminal_mode_send = 2
        print("Start to send miniterminal data.")
    else:
        mrd.flag_tarminal_mode_send = 0
        print("Stop to send miniterminal data.")
        for i in range(8):
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_minitermnal_keep[i][0] = -1
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_minitermnal_keep[i][1] = 0


def set_send_virtual():  # チェックボックスに従いデータ送信フラグをオンオフ
    if mrd.flag_send_virtual == 0:
        mrd.flag_send_virtual = 1
        print("Start noting. Virtual-Hard Uninplemented.")
    else:
        mrd.flag_send_virtual = 0
        print("Quit nothing. Virtual-Hard Uninplemented")


def ros1_pub():  # チェックボックスに従いROS1パブリッシュフラグをオンオフ
    # print("ROS1 is not available.")
    if mrd.flag_ros1_pub == 0:
        mrd.flag_ros1_pub = 1
        print("Start publishing ROS1 joint_states.")
    else:
        mrd.flag_ros1_pub = 0
        print("Quit publishing ROS1 joint_states.")


def ros1_sub():  # チェックボックスに従いROS1サブスクライブフラグをオンオフ
    if mrd.flag_ros1_sub == 0:
        mrd.flag_ros1_sub = 1
        print("Start subscribing ROS1 joint_states.")
    else:
        mrd.flag_ros1_sub = 0
        print("Quit publishing ROS1 joint_states.")


def set_servo_angle(channel, app_data):
    if channel[3] == "L":
        mrd.s_meridim_motion[int(channel[4:6])*2+21] = int(app_data*100)
        print(f"L meri: {int(channel[4:6])*2+21}")
    if channel[3] == "R":
        mrd.s_meridim_motion[int(channel[4:6])*2+51] = int(app_data*100)
        print(f"R meri: {int(channel[4:6])*2+51}")

    print(f"channel is: {channel[3]}")
    print(f"channel is: {channel[4:6]}")
    print(f"app_data is: {int(app_data*100)}")
    print(f"motion is: {mrd.s_meridim_motion[int(channel[4:6])+21]}")


def set_servo_home():
    if mrd.flag_servo_home == 0:
        mrd.flag_servo_home = 1
        print("Set all servo position zero.")
    else:
        mrd.flag_servo_home = 0

# def set_transaction_mode(sender, app_data):
#    if app_data == "Flow":  # ボタン押下でset_flowフラグをオン
#        mrd.flag_set_flow = True
#        mrd.flag_flow_switch = True
#        #mrd.flag_set_step = False
#        print("Set flow to Meridian.")
#    elif app_data == "Step":  # ボタン押下でset_stepフラグをオン
#        mrd.flag_set_flow = False
#        mrd.flag_flow_switch = True
#        #mrd.flag_set_step = True
#        print("Set step to Meridian.")

# def send_data_step_frame():  # チェックボックスに従いアクション送信フラグをオンオフ
#    mrd.flag_send_data_step_frame = True
#    print("Return: Send data and step to the next frame.")


def joinstate_to_meridim(JointState):
    for i in range(11):
        mrd.s_meridim_js_sub[21+i *
                             2] = round(math.degrees(JointState.position[i])*100)*mrd.jspn[i]
        mrd.s_meridim_js_sub[51+i * 2] = round(math.degrees(
            JointState.position[11+i])*100)*mrd.jspn[15+i]


UDP_SEND_IP = get_udp_send_ip()


################################################################################################################################
## dearpyguiによるコンソール画面描写　##############################################################################################
################################################################################################################################

def main():

    # dpg用関数 ======================================================================================================================
    def set_yaw_center():  # IMUのヨー軸センターリセットフラグを10上げる（コマンドを10回送信する）
        mrd.flag_update_yaw = command_send_trial

    def reset_counter():  # カウンターのリセット
        mrd.loop_count = 1
        mrd.error_count_pc_to_esp = 0
        mrd.error_count_esp_to_tsy = 0
        mrd.error_count_tsy_to_esp = 0
        mrd.error_count_esp_to_pc = 0
        mrd.error_count_tsy_skip = 0
        mrd.error_count_esp_skip = 0
        mrd.error_count_pc_skip = 0
        mrd.error_count_servo_skip = 0
        mrd.error_servo_id = "None"
        mrd.start = time.time()

    while (True):

        # dpg描画 ====================================================================================================================
        dpg.create_context()
        dpg.create_viewport(title=TITLE_VERSION, width=870, height=580)

        # （画面上段左側）サーボ位置モニタリング用のウィンドウ ===============================================================================
        with dpg.window(label="Axis Monitor", width=250, height=370, pos=[5, 5]):
            with dpg.group(label='RightSide'):
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID R"+str(i), label="R"+str(
                        i), max_value=100, min_value=-100, callback=set_servo_angle, pos=[10, 35+i*20], width=80)
            with dpg.group(label='LeftSide'):
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID L"+str(i), label="L"+str(
                        i), max_value=100, min_value=-100, callback=set_servo_angle, pos=[135, 35+i*20], width=80)

            dpg.add_button(label="Home",  callback=set_servo_home, pos=[
                           10, 340])  # Sendと書いてあるボタンをwindowの右下に設置

        # （画面下段左側）メッセージ表示用ウィンドウ（アドレス・通信エラー等） ==================================================================
        with dpg.window(label="Messege", width=590, height=155, pos=[5, 380]):
            dpg.add_button(label="ResetCounter",
                           callback=reset_counter, width=90, pos=[470, 30])
            dpg.add_text(mrd.message0, tag="DispMessage0")
            dpg.add_text(mrd.message1, tag="DispMessage1")
            dpg.add_text(mrd.message2, tag="DispMessage2")
            dpg.add_text(mrd.message3, tag="DispMessage3")
            dpg.add_text(mrd.message4, tag="DispMessage4")

        # （画面上段中央）センサー値モニタリング用ウィンドウ =================================================================================
        with dpg.window(label="Sensor Monitor", width=335, height=175, pos=[260, 5]):
            with dpg.group(label='LeftSide'):
                dpg.add_slider_float(default_value=0, tag="mpu0", label="ac_x",
                                     max_value=327, min_value=-327, pos=[10, 35], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu1", label="ac_y",
                                     max_value=327, min_value=-327, pos=[115, 35], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu2", label="ac_z",
                                     max_value=327, min_value=-327, pos=[220, 35], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu3", label="gr_x",
                                     max_value=327, min_value=-327, pos=[10, 55], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu4", label="gr_y",
                                     max_value=327, min_value=-327, pos=[115, 55], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu5", label="gr_z",
                                     max_value=327, min_value=-327, pos=[220, 55], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu6", label="mg_x",
                                     max_value=327, min_value=-327, pos=[10, 75], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu7", label="mg_y",
                                     max_value=327, min_value=-327, pos=[115, 75], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu8", label="mg_z",
                                     max_value=327, min_value=-327, pos=[220, 75], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu9", label="_temp_int16",
                                     max_value=327, min_value=-327, pos=[10, 95], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu10", label="rol",
                                     max_value=327, min_value=-327, pos=[10, 120], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu11", label="pit",
                                     max_value=327, min_value=-327, pos=[115, 120], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu12", label="yaw",
                                     max_value=327, min_value=-327, pos=[220, 120], width=60)
                dpg.add_button(
                    label="SetYaw",  callback=set_yaw_center, width=50, pos=[270, 148])

        # （画面上段右側）リモコン入力コンパネ送信 ==========================================================================================
        with dpg.window(label="Button Input", width=248, height=155, pos=[600, 5]):
            # with dpg.group(label='LeftSide'):
            dpg.add_checkbox(
                tag="Btn_L2",  callback=pad_btn_panel_on, user_data=256, pos=[15, 38])
            dpg.add_checkbox(
                tag="Btn_L1",  callback=pad_btn_panel_on, user_data=1024, pos=[15, 60])
            dpg.add_checkbox(
                tag="Btn_L_UP",  callback=pad_btn_panel_on, user_data=16, pos=[42, 80])
            dpg.add_checkbox(
                tag="Btn_L_DOWN",  callback=pad_btn_panel_on, user_data=64, pos=[42, 124])
            dpg.add_checkbox(
                tag="Btn_L_LEFT",  callback=pad_btn_panel_on, user_data=128, pos=[20, 102])
            dpg.add_checkbox(
                tag="Btn_L_RIGHT",  callback=pad_btn_panel_on, user_data=32, pos=[64, 102])

            dpg.add_checkbox(
                tag="Btn_SELECT",  callback=pad_btn_panel_on, user_data=1, pos=[100, 102])
            dpg.add_checkbox(
                tag="Btn_START",  callback=pad_btn_panel_on, user_data=8, pos=[130, 102])

            dpg.add_checkbox(
                tag="Btn_R2",  callback=pad_btn_panel_on, user_data=512, pos=[215, 38])
            dpg.add_checkbox(
                tag="Btn_R1",  callback=pad_btn_panel_on, user_data=2048, pos=[215, 60])
            dpg.add_checkbox(
                tag="Btn_R_UP",  callback=pad_btn_panel_on, user_data=4096, pos=[188, 80])
            dpg.add_checkbox(
                tag="Btn_R_DOWN",  callback=pad_btn_panel_on, user_data=16384, pos=[188, 124])
            dpg.add_checkbox(
                tag="Btn_R_LEFT",  callback=pad_btn_panel_on, user_data=32768, pos=[166, 102])
            dpg.add_checkbox(
                tag="Btn_R_RIGHT",  callback=pad_btn_panel_on, user_data=8192, pos=[210, 102])

        # （画面中段右側）コマンド送信用ミニターミナル =======================================================================================
        with dpg.window(label="Mini Terminal", width=248, height=203, pos=[600, 165]):
            # with dpg.group(label='LeftSide'):
            dpg.add_text("Index", pos=[15, 25])
            dpg.add_text("Data", pos=[60, 25])
            dpg.add_input_text(tag="s_index0", decimal=True,
                               default_value="0", width=40, pos=[15, 45])
            dpg.add_input_text(tag="s_data0", decimal=True, default_value=str(
                MSG_SIZE), width=60, pos=[60, 45])
            dpg.add_input_text(tag="s_index1", decimal=True,
                               default_value="", width=40, pos=[15, 70])
            dpg.add_input_text(tag="s_data1", decimal=True,
                               default_value="", width=60, pos=[60, 70])
            dpg.add_input_text(tag="s_index2", decimal=True,
                               default_value="", width=40, pos=[15, 95])
            dpg.add_input_text(tag="s_data2", decimal=True,
                               default_value="", width=60, pos=[60, 95])
            dpg.add_input_text(tag="s_index3", decimal=True,
                               default_value="", width=40, pos=[15, 120])
            dpg.add_input_text(tag="s_data3", decimal=True,
                               default_value="", width=60, pos=[60, 120])

            dpg.add_text("Index", pos=[130, 25])
            dpg.add_text("Data", pos=[175, 25])
            dpg.add_input_text(tag="s_index4", decimal=True,
                               default_value="", width=40, pos=[130, 45])
            dpg.add_input_text(tag="s_data4", decimal=True,
                               default_value="", width=60, pos=[175, 45])
            dpg.add_input_text(tag="s_index5", decimal=True,
                               default_value="", width=40, pos=[130, 70])
            dpg.add_input_text(tag="s_data5", decimal=True,
                               default_value="", width=60, pos=[175, 70])
            dpg.add_input_text(tag="s_index6", decimal=True,
                               default_value="", width=40, pos=[130, 95])
            dpg.add_input_text(tag="s_data6", decimal=True,
                               default_value="", width=60, pos=[175, 95])
            dpg.add_input_text(tag="s_index7", decimal=True,
                               default_value="", width=40, pos=[130, 120])
            dpg.add_input_text(tag="s_data7", decimal=True,
                               default_value="", width=60, pos=[175, 120])

            dpg.add_button(label="Set",  callback=set_miniterminal_data, pos=[
                           150, 148])  # Sendと書いてあるボタンをwindowの右下に設置
            dpg.add_text("Send", pos=[184, 148])
            dpg.add_checkbox(tag="TarminalMode",
                             callback=set_tarminal_send_on, pos=[215, 148])
            # dpg.add_radio_button(["Flow", "Step"], tag="transaction_mode", pos=[10, 148], callback=set_transaction_mode, horizontal=True)
            # dpg.add_button(label="Return", pos=[183, 175], callback=send_data_step_frame)  # 右下に設置

        # （画面中段中央）コマンド送信/リモコン値表示用ウィンドウ ===============================================================================
        with dpg.window(label="Command", width=335, height=190, pos=[260, 185]):
            dpg.add_checkbox(label="Power", tag="Power",
                             callback=set_servo_power, pos=[8, 50])
            dpg.add_checkbox(
                tag="Receive",  callback=set_resv_data, pos=[160, 27])
            dpg.add_text("ESP32->", pos=[100, 27])
            dpg.add_checkbox(
                tag="Send",  callback=set_send_data, pos=[160, 50])
            dpg.add_text("ESP32<-", pos=[100, 50])

            dpg.add_checkbox(label="->ROS1", tag="ROS1pub",
                             callback=ros1_pub, pos=[192, 27])
            dpg.add_checkbox(label="<-ROS1", tag="ROS1sub",
                             callback=ros1_sub, pos=[192, 50])
            dpg.add_checkbox(label="<-Demo", tag="Action",
                             callback=set_demo_action, pos=[192, 73])

            dpg.add_text("Control Pad Monitor", pos=[10, 100])
            dpg.add_text("button", tag="pad_button", pos=[170, 100])
            dpg.add_slider_int(default_value=0, tag="pad_Lx", label="Lx",
                               max_value=127, min_value=-127, pos=[10, 120], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Ly", label="Ly",
                               max_value=127, min_value=-127, pos=[90, 120], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Rx", label="Rx",
                               max_value=127, min_value=-127, pos=[170, 120], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Ry", label="Ry",
                               max_value=127, min_value=-127, pos=[250, 120], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_L2v", label="L2v",
                               max_value=255, min_value=0, pos=[90, 140], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_R2v", label="R2v",
                               max_value=255, min_value=0, pos=[170, 140], width=40)

        # dpg変数値の登録
        with dpg.value_registry():
            dpg.add_int_value(tag="button_data")

        dpg.setup_dearpygui()
        dpg.show_viewport()

        # dpg 描 画 内 容 の デ ー タ 更 新 =================================================================================================
        while dpg.is_dearpygui_running():
            signal.signal(signal.SIGINT, signal.SIG_DFL)

            # メッセージ欄の表示更新
            dpg.set_value("DispMessage0", mrd.message0)
            dpg.set_value("DispMessage1", mrd.message1)
            dpg.set_value("DispMessage2", mrd.message2)
            dpg.set_value("DispMessage3", mrd.message3)
            dpg.set_value("DispMessage4", mrd.message4)

            # サーボデータとIMUデータの表示更新
            for i in range(0, 15, 1):
                _idld = mrd.r_meridim[21+i*2]
                _idrd = mrd.r_meridim[51+i*2]
                _idsensor = mrd.r_meridim[i+2]/10000
                dpg.set_value("ID L"+str(i), _idld/100)  # サーボIDと数値の表示
                dpg.set_value("ID R"+str(i), _idrd/100)

                if i < 13:  # IMUデータの更新
                    if i < 11:
                        dpg.set_value("mpu"+str(i), _idsensor)
                    else:
                        dpg.set_value("mpu"+str(i), _idsensor*100)

            # リモコンデータの表示更新
            pad_button_short = np.array([0], dtype=np.uint16)
            # 受信値とコンソール入力値を合成
            pad_button_short[0] = mrd.r_meridim[15] | mrd.pad_button_panel_short[0]
            dpg.set_value("pad_button", str(pad_button_short[0]))
            dpg.set_value("pad_Lx", int(mrd.r_meridim_char[33]))
            dpg.set_value("pad_Ly", int(mrd.r_meridim_char[32]))
            dpg.set_value("pad_Rx", int(mrd.r_meridim_char[35]))
            dpg.set_value("pad_Ry", int(mrd.r_meridim_char[34]))
            _padL2val = (mrd.r_meridim_char[37])
            if (_padL2val < 0):
                _padL2val = 256+_padL2val
            if (mrd.r_meridim[15] & 256 == 0):
                _padL2val = 0
            _padR2val = (mrd.r_meridim_char[36])
            if (_padR2val < 0):
                _padR2val = 256+_padR2val
            if (mrd.r_meridim[15] & 512 == 0):
                _padR2val = 0
            dpg.set_value("pad_L2v", int(_padL2val))
            dpg.set_value("pad_R2v", int(_padR2val))
            dpg.set_value("button_data", int(mrd.r_meridim[15]))

            # ROS1 joint_statesのパブリッシュ ===============================================================================================
            if mrd.flag_ros1_pub:  # ROS送信:joint_statesのpublishを実施
                if rospy_imported:
                    if mrd.flag_ros1 == 0:
                        rospy.init_node('joint_state_meridim', anonymous=True)
                        flag_ros1 = 1
                    joint_pub = rospy.Publisher(
                        'joint_states', JointState, queue_size=10)
                    rate = rospy.Rate(100)  # 100hz
                    js_meridim = JointState()
                    js_meridim.header.stamp = rospy.Time.now()
                    js_meridim.name =\
                        ['c_head_yaw',                      'l_shoulder_pitch',                'l_shoulder_roll',                  'l_elbow_yaw',
                            'l_elbow_pitch',                 'l_hipjoint_yaw',                  'l_hipjoint_roll',                  'l_hipjoint_pitch',
                         'l_knee_pitch',              'l_ankle_pitch',                   'l_ankle_roll',
                         'c_chest_yaw',            'r_shoulder_pitch',                'r_shoulder_roll',                  'r_elbow_yaw',
                         'r_elbow_pitch',     'r_hipjoint_yaw',                  'r_hipjoint_roll',                  'r_hipjoint_pitch',
                         'r_knee_pitch',  'r_ankle_pitch',                   'r_ankle_roll']
                    js_meridim.position = \
                        [math.radians(mrd.s_meridim_motion[21]/100*mrd.jspn[0]), math.radians(mrd.s_meridim_motion[23]/100*mrd.jspn[1]), math.radians(mrd.s_meridim_motion[25]/100)*mrd.jspn[2], math.radians(mrd.s_meridim_motion[27]/100*mrd.jspn[3]),
                            math.radians(mrd.s_meridim_motion[29]/100*mrd.jspn[4]), math.radians(mrd.s_meridim_motion[31]/100*mrd.jspn[5]), math.radians(
                                mrd.s_meridim_motion[33]/100*mrd.jspn[6]), math.radians(mrd.s_meridim_motion[35]/100*mrd.jspn[7]),
                         math.radians(mrd.s_meridim_motion[37]/100*mrd.jspn[8]), math.radians(
                            mrd.s_meridim_motion[39]/100*mrd.jspn[9]), math.radians(mrd.s_meridim_motion[41]/100*mrd.jspn[10]),
                         math.radians(mrd.s_meridim_motion[51]/100*mrd.jspn[15]), math.radians(mrd.s_meridim_motion[53]/100*mrd.jspn[16]), math.radians(
                            mrd.s_meridim_motion[55]/100*mrd.jspn[17]), math.radians(mrd.s_meridim_motion[57]/100*mrd.jspn[18]),
                         math.radians(mrd.s_meridim_motion[59]/100*mrd.jspn[19]), math.radians(mrd.s_meridim_motion[61]/100*mrd.jspn[20]), math.radians(
                            mrd.s_meridim_motion[63]/100*mrd.jspn[21]), math.radians(mrd.s_meridim_motion[65]/100*mrd.jspn[22]),
                         math.radians(mrd.s_meridim_motion[67]/100*mrd.jspn[23]),  math.radians(mrd.s_meridim_motion[69]/100*mrd.jspn[24]), math.radians(mrd.s_meridim_motion[71]/100*mrd.jspn[25])]

                    js_meridim.velocity = []
                    # js_meridim.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    js_meridim.effort = []
                    joint_pub.publish(js_meridim)
                    rate.sleep()
                else:
                    print("ROS is not avaliable.")

            # R O S 1 joint_statesのサブスクライブ ===============================================================================================
            if mrd.flag_ros1_sub:
                if rospy_imported:
                    # joint_sub = rospy.Subscriber('joint_states', JointState, queue_size=10)
                    if flag_ros1 == 0:
                        rospy.init_node('joint_state_meridim', anonymous=True)
                        flag_ros1 = 1
                    rospy.Subscriber('joint_states', JointState,
                                     joinstate_to_meridim)
                    # rospy.spin()
                else:
                    print("ROS is not avaliable.")

            # =================================================================================================================================

            # dpg表示更新処理
            dpg.render_dearpygui_frame()

            time.sleep(0.003)  # CPUの負荷を下げる
        dpg.destroy_context()


# スレッド2つで送受信と画面描写を並列処理
if __name__ == '__main__':
    # dearpyguiによるコンソール画面描写スレッド
    thread1 = threading.Thread(target=meridian_loop)
    thread1.start()
    main()
