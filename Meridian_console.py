# !/usr/bin/python3
# coding: UTF-8
# もしくは # !/usr/bin/env python など環境に合わせて

# Izumi Ninagawa & Meridian project
# MIT License

# 2023.12.30 フロー送信/ステップ送信の切り替え,データ表示のターゲット/実行結果の切り替え,自身のIPの自動取得
# 2023.12.30 これまでのグローバル変数をクラスに格納. 一部をnumpy化. sleepを入れてCPU負荷を軽減
# 2023.12.31 エラー受信値のオーバーフローバグを修正
# 2024.01.05 起動時のモードを"Actual"に修正
# 2024.01.05 ミニターミナルにset&sendを追加し, 送信方法を変更. 【Mini Terminal】参照
# 2024.01.07 ターミナルに送受信データを表示する機能を追加.
# 2024.04.30 L2R2ボタンのアナログ値の表示を修正.
# 2025.01.26 genesis用のRedis入力を追加. POWERとRedisをチェックで動作.
# 2025.04.06 マイコンボードのwifiIPアドレスをboard_ip.txtで設定するように変更.
# 2025.04.29 Trimモードを追加. 
# 2025.06.13 有線LAN,固定IP等選択モード追加. (IP値の入力時にEnterキーで既存の値を使用できます)
# 2026.06.30 TrimモードのUIを更新.

# Meridian console 取扱説明書
#
# ・起動方法
# 当ファイルがあるディレクトリにて, ターミナルより
# python3 Meridian_console.py [送信先のESP32のIPアドレス 例:192.168.1.12]
# と入力して実行します. 必要に応じてライブラリをpip3で追加してください
# (IPアドレスなしで実行した場合は, 82行目のUDP_SEND_IP_DEFでの設定が反映されます)
# UDP_SEND_IPはESP32の起動時にPCシリアルモニタ上に表示されます
#
# ・各ウィンドウについて
# 【Axis Monitor】
# 各サーボの値です. パワーオン時にはスライダでサーボを動かすことができます
# Target/Actualのラジオボタンで, データの表示を送信データ/受信データに切り替えられます
# [HOME]ボタン : サーボの位置をゼロリセットする
# [TRIM]ボタン : サーボのトリム値を設定するTrim Setting Windowを開く(後述)
#
# 【Sensor Monitor】
# MIUのデータを表示します. rol,pit,yawはセンサフュージョン値です. SetYawボタンでヨー軸の中央値をリセットできます
#
# 【Command】
# ◻︎ POWER : 全サーボのパワーをオンオフします
# ◻︎ DEMO  : サインカーブの全身モーションを計算します(<-ESP32オンで送信)
# ◻︎ Python: ユーザー定義のpythonコードを反映します
# ◻︎ Enable: Demoやpythonを送信に反映します
# ◻︎ ->ROS1: ROS1のjointデータをパブリッシュします(Rvisと連動できます)
# ◻︎ <-ROS1: ROS1のサブスクライブします(動作未確認)
# # Control Pad Monitor: リモコンの入力状態を標準化して表示します
#
# 【Mini Terminal】
# Meridim配列のデータをインプットし8つまで同時送信することができます
# MeridianのIndex(Meridim90であれば0~89)とDataを入力し,
# Set&Sendボタンでデータを1回送信します.
# Setを押した後, Continuousチェックを入れることで同じデータを毎フレーム送信します.
# Indexの範囲外のデータは無効となり送信されません. また, チェックを外した時に送信バッファの各Indexに-1が代入されます
# ◯Flow ◯Step : フローモードとステップモードを切り替えます
# [Next frame]: ステップモード時に1フレームだけ次に進みます
#
# 【Button Input】
# コンソールからリモコンボタン押下情報を送信します
#
# 【Message】
# IPと各経路のエラーカウント, エラー率, フレーム数, 動作周波数を表示します
# ResetCycle: 周波数のカウントをリセットするボタンです
# ResetCounter: カウンタの値をリセットするボタンです
# disp_send: 送信データをターミナルに表示するチェックボックスです
# disp_rcvd: 受信データをターミナルに表示するチェックボックスです
# TsySKIP, PcSKIP: 連番データの取りこぼし数を表示します
# PS4リモコン接続時に受信スキップ回数が5%ほど検出されるのは, 現在の仕様では正常な動作です

# 【Trim Setting Window】(2025.05.04現在, LITE版のhotfix/v1.1.1ブランチのみで有効)
# 以下の手順でサーボのトリム値を設定します
# サーボ設定はEEPROMに保存されます
#
# 1.Power, Python, Enable の3箇所すべてにチェックを入れます
# 2.[Start Trim Setting] ボタンを押します
#   Board側のトリム値が0リセットされ, ウィンドウのスライダーにトリム値が反映されます
# 3.実機を見ながら, トリム値を入力します
#   スライダーもしくはインプットフィールドへの入力+Enter,もしくは-+ボタンでトリム値を調整します
# 4.調整できたら,[Save to EEPROM]ボタンを押します. Board側にデータが送信され,EEPROMに登録されます
# 5.[Load EEPROM to Board RAM]ボタンを押します. EEPROMからBoardにトリム値などが反映します
#   この時, ロボット実機がトリムポーズではない位置に動く場合があります
# 6.[Home]ボタンを押すと, サーボは新しいトリム値を基準とした0位置に移動します(トリムポーズ)
# 7.[Export Settings]ボタンを押すと, 最後にEEPROMに読み書きしたトリムデータがテキスト形式で出力されます
#   出力ディレクトリはこのpythonコードと同じ場所になります. ファイル内容はMeridianのBoard用コードにコピペできる形式です
# 8.設定が完了したら[close]ボタンを押して閉じます
#
# 9.実機の起動時に設定値が有効となるようにするには, Board用コードのconfig.hの設定を下記のようにします
#   #define EEPROM_SET     0
#   #define EEPROM_LOAD    1
# 以上により, 実機の起動時にEEPROMの設定値が読み込まれるようになります

import sys
import numpy as np
import socket
import select as _select
from contextlib import closing
import struct
import math
import dearpygui.dearpygui as dpg
import threading
import signal
import time
import atexit
import os
import re
import valkey as redis  # Valkey用ライブラリ (valkey.Valkey = redis.Redis 互換)
import subprocess

# Gamepad support via pygame (headless, no SDL window)
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
try:
    import pygame as _pygame
    _pygame.display.init()
    _pygame.joystick.init()
    _pygame_available = True
except Exception:
    _pygame = None
    _pygame_available = False

# ROS搭載マシンの場合はrospyをインポートする
try:
    import rospy
    rospy_imported = True
    from sensor_msgs.msg import JointState
except ImportError:
    rospy_imported = False
    print("rospy not found. ROS functions will be disabled.")

# UTF-8 モード
sys.stdout.reconfigure(encoding='utf-8')

# 定数
TITLE_VERSION = "Meridian_Console_v26.0702" # DPGのウィンドウタイトル兼バージョン表示
UDP_RECV_PORT = 22222                       # 受信ポート
UDP_SEND_PORT = 22224                       # 送信ポート
MSG_SIZE = 90                               # Meridim配列の長さ(デフォルトは90)
MSG_BUFF = MSG_SIZE * 2                     # Meridim配列のバイト長さ
MSG_ERRS = MSG_SIZE - 2                     # Meridim配列のエラーフラグの格納場所(配列の最後から２番目)
MSG_CKSM = MSG_SIZE - 1                     # Meridim配列のチェックサムの格納場所(配列の末尾)
STEP = 94                                   # 1フレームあたりに増加させる制御処理用の数値,サインカーブを何分割するか
MRD_L_ORIG_IDX = 20                         # Meridim配列のL系統の最初のインデックス(デフォルトは20)
MRD_R_ORIG_IDX = 50                         # Meridim配列のR系統の最初のインデックス(デフォルトは50)
# Meridim配列の1系統あたりの最大接続サーボ数(デフォルトは15)
MRD_SERVO_SLOTS = 15

# Redisサーバー設定
REDIS_HOST = "localhost"
REDIS_PORT = 6379
REDIS_KEY_READ = "meridis_calc_pub"   # <- Redis: 受信キー
REDIS_KEY_WRITE = "meridis_real_pub"  # -> Redis: 送信キー

# マスターコマンド
MRD_MASTER = 0                      # マスターコマンドのMeridim配列での位置
MCMD_TORQUE_ALL_OFF = 0             # すべてのサーボトルクをオフにする(脱力)
MCMD_SENSOR_YAW_CALIB = 10002       # センサの推定ヨー軸を現在値センターとしてリセット
MCMD_SENSOR_ALL_CALIB = 10003       # センサの3軸について現在値を原点としてリセット
MCMD_ERR_CLEAR_SERVO_ID = 10004     # 通信エラーのサーボのIDをクリア(MRD_ERR_l)
MCMD_BOARD_TRANSMIT_ACTIVE = 10005  # ボードが定刻で送信を行うモード(デフォルト設定.PC側が受信待ち)
MCMD_BOARD_TRANSMIT_PASSIVE = 10006  # ボードが受信を待ち返信するモード(PC側が定刻送信)
MCMD_FRAMETIMER_RESET = 10007      # フレームタイマーを現在時刻にリセット
MCMD_BOARD_STOP_DURING = 10008     # ボードの末端処理を[MRD_STOP_FRAMES]ミリ秒止める
MCMD_START_TRIM_SETTING = 10100    # トリム設定モードに入る
MCMD_EEPROM_SAVE_TRIM = 10101       # 現在の姿勢をトリム値としてEEPROMに書き込む
MCMD_EEPROM_LOAD_TRIM = 10102       # EEPROMのトリム値をサーボに反映する
MCMD_EEPROM_BOARDTOPC_DATA0 = 10200  # EEPROMの[0][x]をボードからPCにMeridimで送信する
MCMD_EEPROM_BOARDTOPC_DATA1 = 10201  # EEPROMの[1][x]をボードからPCにMeridimで送信する
MCMD_EEPROM_BOARDTOPC_DATA2 = 10202  # EEPROMの[2][x]をボードからPCにMeridimで送信する
MCMD_EEPROM_PCTOBOARD_DATA0 = 10300  # EEPROMの[0][x]をPCからボードにMeridimで送信する
MCMD_EEPROM_PCTOBOARD_DATA1 = 10301  # EEPROMの[1][x]をPCからボードにMeridimで送信する
MCMD_EEPROM_PCTOBOARD_DATA2 = 10302  # EEPROMの[2][x]をPCからボードにMeridimで送信する

# ================================================================================================================
# ---- Redis クラス (RedisReceiver / RedisTransfer) ---------------------------------------------------------------
# ================================================================================================================

class RedisReceiver:
    """Redisからhash形式でデータを受信するクラス。"""
    def __init__(self, host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_READ,
                 connect_timeout: float = 0.5, socket_timeout: float = 0.5):
        import socket as _socket
        self.host = host
        self.port = port
        self.redis_key = redis_key
        self.is_connected = False
        self._sock_mod = _socket
        self.redis_client = redis.Valkey(
            host=host, port=port, decode_responses=True,
            socket_connect_timeout=connect_timeout,
            socket_timeout=socket_timeout)
        try:
            conn = _socket.create_connection((host, port), timeout=connect_timeout)
            conn.close()
            self.redis_client.ping()
            self.is_connected = True
        except Exception as e:
            print(f"[RedisReceiver] Could not connect: {e}")

    def get_data(self, key=None):
        """指定キーからhash形式で90要素のfloatリストを取得して返す。失敗時はNone。"""
        if not self.is_connected:
            return None
        redis_key = key if key is not None else self.redis_key
        try:
            data = self.redis_client.hgetall(redis_key)
            if not data:
                print(f"[RedisReceiver] No data for key '{redis_key}'.")
                return None
            return [float(data[str(i)]) for i in range(len(data))]
        except (redis.ConnectionError, KeyError, ValueError) as e:
            print(f"[RedisReceiver] Error: {e}")
            return None

    def close(self):
        if self.redis_client:
            try:
                self.redis_client.close()
            except Exception:
                pass


class RedisTransfer:
    """Redisへhash形式でデータを送信するクラス。"""
    def __init__(self, host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_WRITE,
                 connect_timeout: float = 0.5, socket_timeout: float = 0.5):
        import socket as _socket
        self.host = host
        self.port = port
        self.redis_key = redis_key
        self.is_connected = False
        self.redis_client = redis.Valkey(
            host=host, port=port, decode_responses=True,
            socket_connect_timeout=connect_timeout,
            socket_timeout=socket_timeout)
        try:
            conn = _socket.create_connection((host, port), timeout=connect_timeout)
            conn.close()
            self.redis_client.ping()
            self.is_connected = True
            if not self.redis_client.exists(redis_key):
                self.redis_client.hset(redis_key, mapping={str(i): "0" for i in range(90)})
        except Exception as e:
            print(f"[RedisTransfer] Could not connect: {e}")

    def set_data(self, data, key=None):
        """90要素のリストをhash形式でRedisに書き込む。"""
        if not self.is_connected or data is None or len(data) != 90:
            return
        redis_key = key if key is not None else self.redis_key
        try:
            mapping = {str(i): str(float(v)) for i, v in enumerate(data)}
            self.redis_client.hset(redis_key, mapping=mapping)
        except redis.RedisError as e:
            print(f"[RedisTransfer] Error: {e}")

    def close(self):
        if self.redis_client:
            try:
                self.redis_client.close()
            except Exception:
                pass


# ================================================================================================================
# ---- 変数の宣言 -------------------------------------------------------------------------------------------------
# ================================================================================================================


class MeridianConsole:
    def __init__(self):
        # Meridim配列関連
        self.r_meridim = np.zeros(
            MSG_SIZE, dtype=np.int16)            # Meridim配列
        self.s_meridim = np.zeros(
            MSG_SIZE, dtype=np.int16)            # Meridim配列
        self.s_meridim_special = np.zeros(
            MSG_SIZE, dtype=np.int16)    # Meridim配列. 特殊コマンド用
        self.k_meridim_eeprom_last = np.zeros(
            MSG_SIZE, dtype=np.int16)  # Meridim配列. 最新の設定値の保持用
        self.r_meridim_char = np.zeros(
            MSG_SIZE*2, dtype=np.uint8)     # Meridim配列
        self.r_meridim_ushort = np.zeros(
            MSG_SIZE, dtype=np.uint8)     # Meridim配列
        self.d_meridim = np.zeros(MSG_SIZE, dtype=np.int16)            # 表示用
        self.s_meridim_js_sub_f = np.zeros(
            MSG_SIZE, dtype=float)      # ROSサブスクライブ用
        self.pad_button_panel_short = np.array(
            [0], dtype=np.uint16)   # コンパネからのリモコン入力用
        self.pad_gamepad_buttons = np.array([0], dtype=np.uint16)  # ゲームパッドのボタンビットマスク
        self.pad_gamepad_axes = np.zeros(6, dtype=np.float64)      # LX,LY,RX,RY,L2,R2
        self.pad_gamepad_player = -1   # -1=None, 0=Player1, ...
        self.pad_gamepad_connected = False
        self.s_meridim_motion_f = np.zeros(
            MSG_SIZE, dtype=float)      # PC側で作成したサーボ位置送信用
        self.s_meridim_motion_keep_f = np.zeros(
            MSG_SIZE, dtype=float)  # PC側で作成したサーボ位置キープ用
        self.s_miniterminal_keep = np.zeros(
            (8, 2))                     # コンパネからのリモコン入力用
        for i in range(8):
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            self.s_miniterminal_keep[i][0] = -1

        # 終了制御
        self.running = True              # Falseでmeridian_loopを停止

        # エラー集計表示用変数
        self.loop_count = 1              # フレーム数のカウンタ
        self.frame_sync_s = 0            # 送信するframe_sync_r(0-59999)
        self.frame_sync_r_expect = 0     # 毎フレームカウントし, 受信カウントと比較(0-59999)
        self.frame_sync_r_recv = 0       # 今回受信したframe_sync_r
        self.frame_sync_r_last = 0       # 前回受信したframe_sync_r
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
        self.error_servo_id_past = 0     # 前回のサーボエラーIDキープ用

        # 制御コマンド用フラグ等
        self.command_send_trial = 1               # Commandを連続で送信する回数
        # IMUのヨー軸センターリセットフラグ(python内部用)
        self.flag_update_yaw = 0
        self.flag_servo_power = 0                 # 全サーボのパワーオンオフフラグ
        self.flag_udp_resv = True                 # UDP受信の完了フラグ
        self.flag_enable_send_made_data = False   # ESP32への状態データの送信のオンオフフラグ
        self.flag_resv_data = 0                   # ESP32からの状態データの受信のオンオフフラグ
        # ESP32への状態データの送信のオンオフフラグ(サーボオフでも送信可)
        self.flag_send_data = 0
        self.flag_send_virtual = 0                # ハードウェアを接続しないで動作させる場合のバーチャルハードのオンオフフラグ
        self.flag_send_motion = 0                 # 計算モーション送信のオンオフフラグ
        self.flag_set_miniterminal_data = 0       # ミニターミナルの値をセットするボタンのためのフラグ
        self.flag_send_miniterminal_data_cont = 0  # ミニターミナルの値を送信するボタンのためのフラグ
        self.flag_send_miniterminal_data_once = 0  # ミニターミナルの値を1回送信する
        self.flag_special_command_send = 0        # 特殊なコマンドとデータを優先して送信する
        self.flag_terminal_mode_send = 0          # miniterminalを有効にし, コマンドを優先する
        self.flag_demo_action = 0                 # デモ/テスト用の計算モーション送信のオンオフフラグ
        self.flag_python_action = 0               # ユーザー自作python有効のオンオフフラグ
        self.flag_ros1 = 0                        # ROS1の起動init(初回のみ)
        self.flag_ros1_pub = 0                    # ROS1のjoint_statesのパブリッシュ
        self.flag_ros1_sub = 0                    # ROS1のjoint_statesのサブスクライブ
        self.flag_self_mode = False               # Selfモード: UDP受信を無視して100Hzで動作
        self.flag_redis_pub = False               # RedisへのデータパブリッシュON/OFF
        self.flag_redis_sub = False               # Redisデータのサブスクライブ
        self.flag_set_flow_or_step = 1            # Meridianの循環を+:通常フロー, -:ステップ に切り替え
        self.flag_servo_zero = 0                  # 全サーボ位置をゼロリセット
        self.flag_stop_flow = False               # ステップモード中の待機フラグ
        self.flag_allow_flow = False              # ステップモード中に1回データを流すフラグ
        # Axis monitorの表示 1:送信データ(target) 0:受信データ(actual)
        self.flag_display_mode = 0
        # ROS1にパブするデータ 1:送信データ(target) 0:受信データ(actual)
        self.flag_ros1_output_mode = 0
        self.frag_reset_cycle = False             # ボードのフレームの周波数をリセットする
        self.frag_reset_errors = False            # Meridimのエラーカウントをリセットする
        self.flag_disp_send = 0                   # ターミナルに送信データを表示する
        self.flag_disp_rcvd = 0                   # ターミナルに受信データを表示する
        self.flag_trim_window_open = False        # Trimウィンドウの表示状態を管理
        # サーボの回転方向フラグ配列(False=正回転, True=逆回転)
        self.servo_direction = {}
        # サーボのマウント情報(True=マウントあり, False=マウントなし)
        self.servo_mount = {}
        self.servo_id_values = {}                 # サーボIDの情報(0-255)

        for i in range(MRD_SERVO_SLOTS):                        # 右側サーボの初期化
            self.servo_direction[f"L{i}"] = False  # 初期値は正回転(チェックなし)
            self.servo_direction[f"R{i}"] = False  # 初期値は正回転(チェックなし)
            self.servo_mount[f"L{i}"] = True       # 初期値はマウントなし
            self.servo_mount[f"R{i}"] = True       # 初期値はマウントなし
            self.servo_id_values[f"L{i}"] = i      # 初期値はインデックスと同じ
            self.servo_id_values[f"R{i}"] = i      # 初期値はインデックスと同じ

        # メッセージ表示用
        self.message0 = "This PC's IP adress is "+UDP_RECV_IP_DEF
        self.message1 = ""
        self.message2 = ""
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


def get_local_ip():  # 自身のIPアドレスを自動取得する
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # ダミーのsocketを作成
        s.connect(("8.8.8.8", 80))  # GoogleのDNSサーバーに接続を試み,実際には接続しない
        IP = s.getsockname()[0]  # このsocketを通じて取得されるローカルIPアドレスを取得
        s.close()
        return IP
    except Exception as e:
        return "Error: " + str(e)


def check_valid_ip(ip):  # IPアドレスの書式確認
    parts = ip.split(".")
    return (
        len(parts) == 4 and
        all(p.isdigit() and 0 <= int(p) <= 255 for p in parts)
    )


def select_network_mode_and_ip(filename="board_ip.txt"):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, filename)
    # デフォルト値
    default = {
        0: {'SEND': '192.168.3.45', 'RECV': '192.168.3.3'},   # WIFI(DHCP)
        1: {'SEND': '192.168.3.45', 'RECV': '192.168.3.3'},   # WIFI(Fixed)
        2: {'SEND': '192.168.90.1', 'RECV': '192.168.90.2'},  # 有線LAN
    }
    mode_labels = {0: 'WIFI(DHCP)', 1: 'WIFI(Fixed)', 2: 'Wired LAN'}
    config = {}
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            for line in f:
                m = re.match(r'([A-Z_]+)\s*=\s*"?([^"]*)"?', line.strip())
                if m:
                    k, v = m.group(1), m.group(2)
                    config[k] = v
    try:
        network_mode = int(config.get('NETWORK_MODE', '0'))
    except Exception:
        network_mode = 0
    wifi_dhcp_send = config.get(
        'UDP_WIFI_DHCP_SEND_IP_DEF', default[0]['SEND'])
    wifi_dhcp_recv = config.get(
        'UDP_WIFI_DHCP_RECV_IP_DEF', default[0]['RECV'])
    wifi_fixed_send = config.get(
        'UDP_WIFI_FIXED_SEND_IP_DEF', default[1]['SEND'])
    wifi_fixed_recv = config.get(
        'UDP_WIFI_FIXED_RECV_IP_DEF', default[1]['RECV'])
    wired_send = config.get('UDP_WIRED_SEND_IP_DEF', default[2]['SEND'])
    wired_recv = config.get('UDP_WIRED_RECV_IP_DEF', default[2]['RECV'])
    ip_table = {
        0: {'SEND': wifi_dhcp_send, 'RECV': wifi_dhcp_recv},
        1: {'SEND': wifi_fixed_send, 'RECV': wifi_fixed_recv},
        2: {'SEND': wired_send, 'RECV': wired_recv},
    }
    while True:
        print(f"Use previous {mode_labels[network_mode]} settings?")
        print(
            f"SEND_IP: {ip_table[network_mode]['SEND']}, RECV_IP: {ip_table[network_mode]['RECV']}")
        yn = input("y/n (Enter for y): ").strip().lower()
        if yn in ['', 'y', 'yes']:
            break
        elif yn in ['n', 'no']:
            while True:
                print("Please select a mode. \n0:WIFI(DHCP), 1:WIFI(Fixed), 2:Wired LAN")
                mode_in = input(
                    f"Enter mode number (current: {network_mode}): ").strip()
                if mode_in == '':
                    break
                try:
                    mode_in = int(mode_in)
                    if mode_in in [0, 1, 2]:
                        network_mode = mode_in
                        break
                except Exception:
                    pass
                print("Please enter 0, 1, or 2.")
            for key in ['SEND', 'RECV']:
                label = f"Enter the {key} IP for {mode_labels[network_mode]} (current: {ip_table[network_mode][key]}):"
                while True:
                    ip_in = input(label).strip()
                    if ip_in == '':
                        break
                    if check_valid_ip(ip_in):
                        ip_table[network_mode][key] = ip_in
                        break
                    print("Invalid IP address format. Example: 192.168.1.100")
            if network_mode == 0:
                config['UDP_WIFI_DHCP_SEND_IP_DEF'] = ip_table[0]['SEND']
                config['UDP_WIFI_DHCP_RECV_IP_DEF'] = ip_table[0]['RECV']
            elif network_mode == 1:
                config['UDP_WIFI_FIXED_SEND_IP_DEF'] = ip_table[1]['SEND']
                config['UDP_WIFI_FIXED_RECV_IP_DEF'] = ip_table[1]['RECV']
            elif network_mode == 2:
                config['UDP_WIRED_SEND_IP_DEF'] = ip_table[2]['SEND']
                config['UDP_WIRED_RECV_IP_DEF'] = ip_table[2]['RECV']
            config['NETWORK_MODE'] = str(network_mode)
            with open(filepath, 'w') as f:
                f.write(
                    f'UDP_WIFI_DHCP_SEND_IP_DEF="{config.get("UDP_WIFI_DHCP_SEND_IP_DEF", default[0]["SEND"])}"\n')
                f.write(
                    f'UDP_WIFI_DHCP_RECV_IP_DEF="{config.get("UDP_WIFI_DHCP_RECV_IP_DEF", default[0]["RECV"])}"\n')
                f.write(
                    f'UDP_WIFI_FIXED_SEND_IP_DEF="{config.get("UDP_WIFI_FIXED_SEND_IP_DEF", default[1]["SEND"])}"\n')
                f.write(
                    f'UDP_WIFI_FIXED_RECV_IP_DEF="{config.get("UDP_WIFI_FIXED_RECV_IP_DEF", default[1]["RECV"])}"\n')
                f.write(
                    f'UDP_WIRED_SEND_IP_DEF="{config.get("UDP_WIRED_SEND_IP_DEF", default[2]["SEND"])}"\n')
                f.write(
                    f'UDP_WIRED_RECV_IP_DEF="{config.get("UDP_WIRED_RECV_IP_DEF", default[2]["RECV"])}"\n')
                f.write(f'NETWORK_MODE = {network_mode}\n')
            print("Settings saved.\n")
            break  # 設定保存後は即ループを抜けてサービス開始
        else:
            print("Please answer with y or n.")
    return ip_table[network_mode]['SEND'], ip_table[network_mode]['RECV'], network_mode


# Trim Setting ウィンドウを開く
def open_trim_window():
    if not mrd.flag_trim_window_open:
        mrd.flag_trim_window_open = True
        print("Open Trim Setting window.")
        create_trim_window()
    else:
        print("Trim Setting window is already open.")

# Trim Setting ウィンドウを閉じる


def close_trim_window():
    if not mrd.flag_trim_window_open:  # on_close二重呼び出しを防ぐガード
        return
    mrd.flag_trim_window_open = False  # ガードフラグを先に下げる
    load_trimdata_from_eeprom_to_board()
    if dpg.does_item_exist("Trim Setting"):
        dpg.delete_item("Trim Setting")
    print("Closed Trim Setting window.")


# Trim Setting側のPowerチェックボックスが変更されたとき, Command側のPowerチェックボックスも同期
def sync_power_from_trim(sender, app_data, user_data):
    dpg.set_value("Power", app_data)
    set_servo_power("Power", app_data, None)


# Trim Setting側のEnableチェックボックスが変更されたとき, Command側のEnableチェックボックスも同期
def sync_enable_from_trim(sender, app_data, user_data):
    dpg.set_value("Enable", app_data)
    set_enable("Enable", app_data, None)


# インプットフィールドの値をスライダーに適用する
def apply_input_value(sender, app_data, user_data):
    # user_dataからサーボIDを取得(例："R1"や"L3"など)
    servo_ix = user_data
    input_tag = "Input " + servo_ix
    slider_tag = "ID " + servo_ix

    try:
        input_value = dpg.get_value(input_tag)  # インプットフィールドから値を取得
        if input_value == "":
            return

        value = float(input_value)         # 文字列を浮動小数点数に変換
        value = round(value, 2)            # 小数点2桁までに制限
        value = max(-100, min(100, value))  # 範囲を制限 (-100 から 100)
        dpg.set_value(slider_tag, value)   # スライダーに値を設定
        set_servo_angle(slider_tag, value)  # スライダーの処理を呼び出してサーボ角度を設定
        dpg.set_value(input_tag, "")       # 入力フィールドをクリア

    except ValueError:
        dpg.set_value(input_tag, "")       # 数値以外が入力された場合は何もしない
        print(f"Invalid input for servo {servo_ix}. Please enter a number.")


# トリム用インプットフィールドの値をスライダーに適用する
def apply_trim_input_value(sender, app_data, user_data):
    # user_dataからサーボID(例："R1"や"L3"など)を取得
    servo_ix = user_data
    input_tag = f"Input_Trim_{servo_ix}"
    slider_tag = f"Trim_{servo_ix}"
    axis_slider_tag = f"ID {servo_ix}"

    try:
        # インプットフィールドから値を取得
        input_value = dpg.get_value(input_tag)
        if input_value == "":
            return

        value = float(input_value)              # 文字列を浮動小数点数に変換
        value = round(value, 2)                 # 小数点2桁までに制限
        value = max(-100, min(100, value))      # 範囲を制限 (-100 から 100)
        dpg.set_value(slider_tag, value)        # Trim Settingのスライダーに値を設定
        dpg.set_value(axis_slider_tag, value)   # Axis Monitorのスライダーにも値を設定
        set_servo_angle(axis_slider_tag, value)  # サーボ角度を設定
        dpg.set_value(input_tag, "")            # 入力フィールドをクリア

    except ValueError:
        dpg.set_value(input_tag, "")            # 数値以外が入力された場合は何もしない
        print(f"Invalid input for servo {servo_ix}. Please enter a number.")


# EEPROMへの保存
def save_trimdata_to_eeprom():
    # Board側にサーボのトリム値と設定(ID, マウント, 回転方向)をEEPROMに保存させる

    trim_msg = "Send Trim data:\n"

    for side, orig_idx in [("L", MRD_L_ORIG_IDX), ("R", MRD_R_ORIG_IDX)]:
        for i in range(MRD_SERVO_SLOTS):
            servo_ix = f"{side}{i}"
            settings = 0  # 設定値をゼロから構築

            # マウント情報 (bit0)
            if mrd.servo_mount[servo_ix]:
                settings |= 0x01

            # サーボID (bit1-7)
            servo_id_val = mrd.servo_id_values[servo_ix] & 0x7F  # 7ビットに制限
            settings |= (servo_id_val << 1)

            # 回転方向 (bit8)
            if not mrd.servo_direction[servo_ix]:  # 逆転の場合はビット8を立てる
                settings |= 0x100

            # int16の範囲内に収まるように調整
            if settings > 32767:
                settings -= 65536

            # 更新したサーボ設定を格納する
            mrd.s_meridim_special[orig_idx + i * 2] = settings

            # トリム値の設定
            trim_val = 0.0
            if mrd.flag_trim_window_open and dpg.does_item_exist(f"Trim_{servo_ix}"):
                trim_val = dpg.get_value(f"Trim_{servo_ix}")
                mrd.s_meridim_special[orig_idx + 1 + i * 2] = int(trim_val * 100)

            trim_msg += servo_ix + " " + f"{trim_val:.2f}"
            if i < MRD_SERVO_SLOTS - 1:                 # ラスト以外はカンマ区切り
                trim_msg += ", "
        trim_msg += "\n"

    print(trim_msg)

    # マスターコマンドを設定
    mrd.s_meridim_special[MRD_MASTER] = MCMD_EEPROM_SAVE_TRIM

    # 特殊コマンド送信のフラグを立てる
    mrd.flag_special_command_send = 1

    print("Command sent: Save trim and settings to EEPROM (10101)")
    set_trim_step_highlight(2)  # Step3を一時ハイライト(保存中), チェーン完了後にStep2へ戻る

    # SAVE_TRIM送信後、表示スライダーを0にリセット
    # サーボはLOAD_TRIM(10102)チェーンでTrim Zeroへ移動させる([6-2]参照)
    for i in range(MRD_SERVO_SLOTS):
        for side in ("L", "R"):
            dpg.set_value(f"ID {side}{i}", 0)
            if mrd.flag_trim_window_open and dpg.does_item_exist(f"Trim_{side}{i}"):
                dpg.set_value(f"Trim_{side}{i}", 0)


# EEPROMからBoardへデータを読み込ませる ####
def load_trimdata_from_eeprom_to_board():
    # Meridimのマスターコマンド10102を送信するための処理

    # Meridim配列を受信値で初期化
    # mrd.s_meridim_special = mrd.r_meridim

    # 特殊コマンドのデータ用のMeridim配列を初期化
    mrd.s_meridim_special = np.zeros(MSG_SIZE, dtype=np.int16)

    # 受信データを特殊コマンド用の配列に転記(現在のサーボ値をそのまま使う)
    for i in range(MSG_SIZE):
        mrd.s_meridim_special[i] = mrd.r_meridim[i]

    # Meridim配列の全サーボ位置に0を入れて送信 ####
    for i in range(MRD_SERVO_SLOTS):
        left_ix = MRD_L_ORIG_IDX + 1 + i * 2
        right_ix = MRD_R_ORIG_IDX + 1 + i * 2

        mrd.s_meridim_special[left_ix] = 0
        mrd.s_meridim_special[right_ix] = 0

    # マスターコマンドを設定
    mrd.s_meridim_special[MRD_MASTER] = MCMD_EEPROM_LOAD_TRIM

    # motion bufferをゼロにして表示ループが0を返すようにする
    for i in range(MRD_SERVO_SLOTS):
        left_ix = MRD_L_ORIG_IDX + 1 + i * 2
        right_ix = MRD_R_ORIG_IDX + 1 + i * 2
        mrd.s_meridim_motion_f[left_ix] = 0
        mrd.s_meridim_motion_f[right_ix] = 0
        mrd.s_meridim_motion_keep_f[left_ix] = 0
        mrd.s_meridim_motion_keep_f[right_ix] = 0

    # 特殊コマンド送信のフラグを立てる
    mrd.flag_special_command_send = 1

    print("Command sent: Load trim data from EEPROM to board (10102)")


# サーボの回転方向フラグを切り替える処理
def toggle_servo_direction(sender, app_data, user_data):
    servo_ix = user_data  # user_dataからサーボID(例："R0"や"L3")を取得

    # フラグの値を更新
    mrd.servo_direction[servo_ix] = app_data

    # チェック時は逆回転, 非チェック時は正回転
    print(
        f"Servo {servo_ix} direction set to {'REVERSE' if app_data else 'NORMAL'}")

    # この値はボード側で読み取られるためだけのもので,
    # このPythonプログラム内でのサーボ制御には影響を与えない


# Trim Setting側のスライダーが動いた時の処理
def set_servo_angle_from_trim(channel, app_data):
    servo_ix = channel.replace("Trim_", "")
    print(f"スライダー設定値: Trim_{servo_ix} = {app_data}")

    # Axis Monitor側のスライダーを物理trim値(正値)で更新
    dpg.set_value(f"ID {servo_ix}", app_data)

    # cw方向を考慮してボードへ送る位置を計算(board trim=0の状態でTrim Zeroに到達するため)
    cw = -1 if mrd.servo_direction.get(servo_ix, False) else 1
    pos = app_data * cw

    if servo_ix.startswith("L"):
        index = int(servo_ix[1:])
        mrd.s_meridim[MRD_L_ORIG_IDX + 1 + index * 2] = int(pos * 100)
        mrd.s_meridim_motion_f[MRD_L_ORIG_IDX + 1 + index * 2] = pos
    elif servo_ix.startswith("R"):
        index = int(servo_ix[1:])
        mrd.s_meridim[MRD_R_ORIG_IDX + 1 + index * 2] = int(pos * 100)
        mrd.s_meridim_motion_f[MRD_R_ORIG_IDX + 1 + index * 2] = pos


def process_eeprom_data():
    """
    EEPROMからのデータを処理し, サーボの設定をUIに反映する
    bit0: マウントの有無(1=マウントあり, 0=マウントなし)
    bit1-7: サーボID(0-255)
    bit8: 回転方向(0=逆転, 1=正転,)
    """
    print("Processing EEPROM data...")

    # 各サーボの設定を処理
    for i in range(MRD_SERVO_SLOTS):
        for side, orig_idx in [("L", MRD_L_ORIG_IDX), ("R", MRD_R_ORIG_IDX)]:
            servo_key = f"{side}{i}"
            settings  = mrd.r_meridim[orig_idx + i * 2]
            trim_val  = mrd.r_meridim[orig_idx + 1 + i * 2] * 0.01  # トリム値(100で割って実際の角度に)

            # マウント情報(bit0)、サーボID(bit1-7)、回転方向(bit8)を抽出
            mount      = (settings & 0x01) > 0
            servo_id   = (settings >> 1) & 0x7F          # 7ビット分のマスク
            is_reverse = ((settings >> 8) & 0x01) == 0   # 0=逆転, 1=正転

            # フラグと値を更新
            mrd.servo_mount[servo_key]      = mount
            mrd.servo_id_values[servo_key]  = servo_id
            mrd.servo_direction[servo_key]  = is_reverse

            print(f"{servo_key} - ID: {servo_id}, Mt: {'1' if mount else '0'}, Dir: {'Rev ' if is_reverse else 'Norm'}, Trim: {trim_val}")

            # Trim Settingウィンドウが開いている場合, UI要素を更新
            if mrd.flag_trim_window_open:
                if dpg.does_item_exist(f"Direction_{servo_key}"):
                    dpg.set_value(f"Direction_{servo_key}", is_reverse)
                if dpg.does_item_exist(f"Mount_{servo_key}"):
                    dpg.set_value(f"Mount_{servo_key}", mount)
                if dpg.does_item_exist(f"ID_{servo_key}"):
                    dpg.set_value(f"ID_{servo_key}", str(servo_id))
                if dpg.does_item_exist(f"Trim_{servo_key}"):
                    dpg.enable_item(f"Trim_{servo_key}")  # disabled状態でもset_valueが反映されるよう先に有効化
                    dpg.set_value(f"Trim_{servo_key}", trim_val)

    # 読み込んだトリム値をサーボ位置にも反映する
    if mrd.flag_servo_power:  # サーボパワーがオンの場合のみ適用
        for i in range(MRD_SERVO_SLOTS):
            for side, orig_idx in [("L", MRD_L_ORIG_IDX), ("R", MRD_R_ORIG_IDX)]:
                servo_key = f"{side}{i}"
                cw        = -1 if mrd.servo_direction.get(servo_key, False) else 1
                trim_val  = mrd.r_meridim[orig_idx + 1 + i * 2] * 0.01
                pos       = trim_val * cw
                mrd.s_meridim[orig_idx + 1 + i * 2]        = int(pos * 100)
                mrd.s_meridim_motion_f[orig_idx + 1 + i * 2] = pos
                dpg.set_value(f"ID {side}{i}", trim_val)   # 表示は物理trim値(正値)

    mrd.k_meridim_eeprom_last = mrd.r_meridim  # 受信したEEPROMの値を出力用にキープ

    print("EEPROM data loaded to Trim Settings window and applied to servos.")


# サーボのマウント有無を切り替える
def toggle_servo_mount(sender, app_data, user_data):
    servo_ix = user_data  # user_dataからサーボID(例："R0"や"L3")を取得
    mrd.servo_mount[servo_ix] = app_data  # フラグの値を更新

    print(
        f"Servo {servo_ix} mount set to {'MOUNTED' if app_data else 'NOT MOUNTED'}")

# サーボIDを設定


def set_servo_id(sender, app_data, user_data):
    servo_key = user_data  # user_dataからサーボキー(例："R0"や"L3")を取得

    try:
        # 入力値をint型に変換
        servo_id = int(app_data)

        # 範囲チェック(0-127)
        if 0 <= servo_id <= 127:
            mrd.servo_id_values[servo_key] = servo_id
            print(f"Servo {servo_key} ID set to {servo_id}")
        else:
            # 範囲外の値の場合は元の値に戻す
            dpg.set_value(sender, str(mrd.servo_id_values[servo_key]))
            print(f"Invalid servo ID. Must be between 0-127.")
    except ValueError:
        # 数値でない場合は元の値に戻す
        dpg.set_value(sender, str(mrd.servo_id_values[servo_key]))
        print(f"Invalid input. Servo ID must be a number.")


# Start Trim Settingボタンの処理
def set_trim_step_highlight(step):
    """ステップインジケーターの枠を更新する (0=Step1, 1=Step2, 2=Step3)"""
    on_color = [100, 180, 255, 220]
    on_fill  = [0, 0, 0, 0]
    off      = [0, 0, 0, 0]
    for i in range(3):
        tag = f"TrimStepHL{i}"
        if dpg.does_item_exist(tag):
            dpg.configure_item(tag, color=on_color if i == step else off,
                               fill=on_fill if i == step else off)
    # Exit highlight: step==2(Save to EEPROM後)に追加表示
    if dpg.does_item_exist("TrimStepHL3"):
        dpg.configure_item("TrimStepHL3",
                           color=on_color if step == 2 else off,
                           fill=on_fill if step == 2 else off)


def set_trim_area_enabled(enabled):
    # オーバーレイの表示切り替え(disabled時に前面に表示してウィジェットを塗りつぶす)
    for _tag in ("TrimCoverOverlay", "TrimCoverToolbarMid", "TrimCoverBottomRow"):
        if dpg.does_item_exist(_tag):
            dpg.configure_item(_tag, show=not enabled)

    for i in range(MRD_SERVO_SLOTS):
        for side in ["R", "L"]:
            for tag in [f"Mount_{side}{i}", f"ID_{side}{i}", f"Direction_{side}{i}",
                        f"Trim_{side}{i}",
                        f"TrimMinus_{side}{i}", f"TrimPlus_{side}{i}",
                        f"Input_Trim_{side}{i}", f"Enter_Trim_{side}{i}"]:
                if dpg.does_item_exist(tag):
                    dpg.enable_item(tag) if enabled else dpg.disable_item(tag)

    if dpg.does_item_exist(STEP_TAG):
        dpg.enable_item(STEP_TAG) if enabled else dpg.disable_item(STEP_TAG)

    set_trim_step_highlight(1 if enabled else 0)


def start_trim_setting():
    # トリム設定モードを開始するために, マスターコマンドとしてMCMD_START_TRIM_SETTINGを送信する
    set_trim_area_enabled(True)

    # 前回取得済みのEEPROMデータがあれば即座にスライダーへ反映する(ボード応答を待たずに表示)
    if int(mrd.k_meridim_eeprom_last[MRD_MASTER]) == MCMD_EEPROM_BOARDTOPC_DATA1:
        data = mrd.k_meridim_eeprom_last
        for i in range(MRD_SERVO_SLOTS):
            for side, orig_idx in [("L", MRD_L_ORIG_IDX), ("R", MRD_R_ORIG_IDX)]:
                servo_key = f"{side}{i}"
                if dpg.does_item_exist(f"Trim_{servo_key}"):
                    dpg.set_value(f"Trim_{servo_key}", data[orig_idx + 1 + i * 2] * 0.01)

    # 特殊コマンドのデータ用のMeridim配列を初期化
    mrd.s_meridim_special = np.zeros(MSG_SIZE, dtype=np.int16)

    # 受信データを特殊コマンド用の配列に転記(現在のサーボ値をそのまま使う)
    for i in range(MSG_SIZE):
        mrd.s_meridim_special[i] = mrd.r_meridim[i]

    # マスターコマンドを設定
    mrd.s_meridim_special[MRD_MASTER] = MCMD_START_TRIM_SETTING

    # 特殊コマンド送信のフラグを立てる
    mrd.flag_special_command_send = 1

    print("Command sent: Start Trim Setting Mode (10100)")

# 整数値から特定の位置と長さでビットを抽出する関数


def mrd_slice_bits(value, pos, length):
    # value: int, np.int32, np.int64など - 元の値
    # pos: int - 抽出を開始するビット位置(0から数える)
    # length: int - 抽出するビットの長さ
    # Returns:int - 抽出されたビット値

    # NumPy配列の場合は各要素に対して処理
    if isinstance(value, np.ndarray):
        # ビットマスクの作成
        mask = (1 << length) - 1
        # シフトとマスクの適用
        return np.bitwise_and(np.right_shift(value, pos), mask)
    else:
        # スカラー値の場合
        return (value >> pos) & ((1 << length) - 1)

# Trim Setting で設定したパラメータをファイルに出力する


def _generate_export_content(for_display=False):
    """Export用のヘッダーファイル文字列を生成して返す。
    for_display=True の場合はヘッダーガードを省略する。"""
    _L = ["Head Yaw", "L Shoulder Pitch", "L Shoulder Roll", "L Elbow Yaw", "L Elbow Pitch",
          "L Hip Yaw", "L Hip Roll", "L Hip Pitch", "L Knee Pitch", "L Ankle Pitch", "L Ankle Roll"]
    _R = ["Waist Yaw", "R Shoulder Pitch", "R Shoulder Roll", "R Elbow Yaw", "R Elbow Pitch",
          "R Hip Yaw", "R Hip Roll", "R Hip Pitch", "R Knee Pitch", "R Ankle Pitch", "R Ankle Roll"]

    def _cmt(i, names):
        name = names[i] if i < len(names) else f"Extra servo {i}"
        return f" // [{i:02d}] {name}"

    def _block(arr_name, type_str, comment, names, value_fn):
        max_macro = "IXL_MAX" if arr_name.startswith("IXL") else "IXR_MAX"
        out = f"// {comment}\n{type_str} {arr_name}[{max_macro}] = {{\n"
        for i in range(MRD_SERVO_SLOTS):
            out += f"{value_fn(i)},{_cmt(i, names)}\n"
        return out + "};\n\n"

    data = mrd.k_meridim_eeprom_last
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    content = f"// Generated by Meridian_Console on {timestamp}\n\n"

    if not for_display:
        content += "#ifndef MERIDIAN_LITE_SERVO_TRIM_CODE_H\n"
        content += "#define MERIDIAN_LITE_SERVO_TRIM_CODE_H\n\n"
        content += "#define IXL_MAX 15\n"
        content += "#define IXR_MAX 15\n\n"

    content += _block("IXL_ID",   "int",   "L-side servo hardware ID for each servo index",
                      _L, lambda i: mrd_slice_bits(data[MRD_L_ORIG_IDX + i * 2], 1, 7))
    content += _block("IXR_ID",   "int",   "R-side servo hardware ID for each servo index",
                      _R, lambda i: mrd_slice_bits(data[MRD_R_ORIG_IDX + i * 2], 1, 7))
    content += _block("IXL_CW",   "int",   "L-side servo rotation direction (1:normal, -1:reverse)",
                      _L, lambda i: 1 if mrd_slice_bits(data[MRD_L_ORIG_IDX + i * 2], 8, 1) else -1)
    content += _block("IXR_CW",   "int",   "R-side servo rotation direction (1:normal, -1:reverse)",
                      _R, lambda i: 1 if mrd_slice_bits(data[MRD_R_ORIG_IDX + i * 2], 8, 1) else -1)
    content += _block("IXL_TRIM", "float", "L-side servo trim values (degree)",
                      _L, lambda i: data[MRD_L_ORIG_IDX + 1 + i * 2] * 0.01)
    content += _block("IXR_TRIM", "float", "R-side servo trim values (degree)",
                      _R, lambda i: data[MRD_R_ORIG_IDX + 1 + i * 2] * 0.01)

    if not for_display:
        content += "#endif // MERIDIAN_LITE_SERVO_TRIM_CODE_H\n"

    return content


def export_settings_to_file():
    """EEPROMの内容を MeridianLiteServoTrimCode.h に保存する。"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, "MeridianLiteServoTrimCode.h")
    try:
        with open(file_path, "w") as f:
            f.write(_generate_export_content())
        print(f"Settings exported to: {file_path}")
        _show_export_modal(file_path, success=True)
        return True
    except Exception as e:
        print(f"Error exporting settings: {str(e)}")
        _show_export_modal(str(e), success=False)
        return False


def show_setting_data_modal():
    """生成されるExportコンテンツをモーダルで表示し、クリップボードへコピーできるようにする。"""
    _TAG = "SettingDataModal"
    if dpg.does_item_exist(_TAG):
        dpg.delete_item(_TAG)
    content = _generate_export_content(for_display=True)
    vw = dpg.get_viewport_width()
    vh = dpg.get_viewport_height()
    w, h = 620, 520
    with dpg.window(label="Setting Data", modal=True, show=True, tag=_TAG,
                    width=w, height=h, no_resize=True,
                    pos=[vw // 2 - w // 2, vh // 2 - h // 2]):
        dpg.add_input_text(tag=f"{_TAG}_text", default_value=content,
                           multiline=True, readonly=True,
                           width=w - 24, height=h - 80)
        dpg.add_spacer(height=6)
        with dpg.group(horizontal=True):
            dpg.add_button(label="Copy All", width=100,
                           callback=lambda: dpg.set_clipboard_text(
                               dpg.get_value(f"{_TAG}_text")))
            dpg.add_spacer(width=10)
            dpg.add_button(label="Close", width=80,
                           callback=lambda: dpg.delete_item(_TAG))


def _show_export_modal(message, success):
    _TAG = "ExportResultModal"
    if dpg.does_item_exist(_TAG):
        dpg.delete_item(_TAG)
    vw = dpg.get_viewport_width()
    vh = dpg.get_viewport_height()
    w, h = 540, 110
    with dpg.window(label="Export Settings", modal=True, show=True, tag=_TAG,
                    width=w, height=h, no_resize=True,
                    pos=[vw // 2 - w // 2, vh // 2 - h // 2]):
        if success:
            dpg.add_text("Exported successfully:")
            dpg.add_text(message, wrap=w - 24)
        else:
            dpg.add_text(f"Export failed: {message}", wrap=w - 24)
        dpg.add_spacer(height=6)
        dpg.add_button(label="Close", width=80,
                       callback=lambda: dpg.delete_item(_TAG))


STEP_TAG = "TrimStep"        # ステップ入力フィールド用タグ


def step_trim(sender, app_data, user_data):
    slider_tag, direction = user_data

    # ステップ値 Δ を取得(負数なら 0 に丸める)
    step_val = max(0.0, float(dpg.get_value("TrimStep") or 0))

    cur_val = float(dpg.get_value(slider_tag))  # 現スライダー値 → new 値をクリップ計算
    new_val = max(-180.0, min(180.0, cur_val + direction * step_val))

    dpg.set_value(slider_tag, new_val)  # スライダー自体の値を更新

    input_tag = "Input_" + slider_tag.replace("Trim_", "")  # 相方 Input フィールドも同期
    if dpg.does_item_exist(input_tag):
        dpg.set_value(input_tag, f"{new_val:.2f}")

    # スライダーでドラッグしたときと同じ処理を手動呼び出し
    set_servo_angle_from_trim(slider_tag, new_val)


# Trim Setting ウィンドウを作成する関数(最新版)
def create_trim_window():

    # ビューポートサイズ取得
    viewport_width = dpg.get_viewport_width()
    viewport_height = dpg.get_viewport_height()

    # ブロック基準位置(画面幅に対する 7/25, 18/25 の比率を採用)
    trim_window_left_block = viewport_width * 7 // 30   # 右サーボ列(R)
    trim_window_right_block = viewport_width * 21 // 30  # 左サーボ列(L)

    with dpg.window(label="Trim Setting", tag="Trim Setting",
                    width=viewport_width-20, height=viewport_height-20,
                    pos=[10, 10], on_close=close_trim_window):

        # --- 上部コントロールエリア -------------------------------------------------
        # Trim窓オープン時: Python を強制OFF
        dpg.set_value("python", False)
        set_python_action("python", False, None)

        # Enable ON 前に受信サーボ値をモーションバッファへ複写し, 位置変化を防ぐ
        for i in range(MRD_SERVO_SLOTS):
            l_ix = MRD_L_ORIG_IDX + 1 + i * 2
            r_ix = MRD_R_ORIG_IDX + 1 + i * 2
            mrd.s_meridim_motion_f[l_ix] = mrd.r_meridim[l_ix] * 0.01
            mrd.s_meridim_motion_f[r_ix] = mrd.r_meridim[r_ix] * 0.01
            mrd.s_meridim_motion_keep_f[l_ix] = mrd.s_meridim_motion_f[l_ix]
            mrd.s_meridim_motion_keep_f[r_ix] = mrd.s_meridim_motion_f[r_ix]

        # Power/Enable を強制ON
        dpg.set_value("Power", True)
        set_servo_power("Power", True, None)
        dpg.set_value("Enable", True)
        set_enable("Enable", True, None)

        # --- Step1 / Step2 / Step3 / Exit (均等4分割) ---
        _s = viewport_width // 4  # 1セクション幅
        dpg.add_text("Step1: ", pos=[15, 35])
        dpg.add_button(label="Enter Trim Mode",
                       callback=start_trim_setting, pos=[60, 35], width=140)

        dpg.add_text("Step2: Adjust Trim to Zero", pos=[_s + 20, 35])

        dpg.add_text("Step3: ", pos=[_s * 2 + 40, 35])
        dpg.add_button(label="Save to EEPROM", callback=save_trimdata_to_eeprom,
                       pos=[_s * 2 + 88, 35], width=125)

        dpg.add_button(label="Exit", callback=close_trim_window,
                       pos=[viewport_width - 140, 35], width=100)

        dpg.add_checkbox(label="Power",  tag="Power_Trim", default_value=True,
                         pos=[15, viewport_height-52], callback=sync_power_from_trim)
        dpg.add_checkbox(label="Enable", tag="Enable_Trim", default_value=True,
                         pos=[80, viewport_height-52], callback=sync_enable_from_trim)

        # --- ステップ値入力フィールド: right edge aligns with L-column Enter button right edge ---
        _delta_right = trim_window_right_block + 197  # trim_window_right_block+155(Enter pos) + 42(width)
        dpg.add_text("delta", pos=[_delta_right - 133, viewport_height - 90])
        dpg.add_input_float(tag=STEP_TAG, label="##delta_label", default_value=1.0, width=90,
                            pos=[_delta_right - 90, viewport_height - 92],
                            min_value=0.0, min_clamped=True, format="%.2f")

        # --- ヘッダー(右サーボ列) -------------------------------------------------
        dpg.add_text("Idx", tag="TrimH_R_Idx", pos=[trim_window_left_block-165, 83])
        dpg.add_text("Mt",  tag="TrimH_R_Mt",  pos=[trim_window_left_block-132, 83])
        dpg.add_text("ID",  tag="TrimH_R_ID",  pos=[trim_window_left_block-104, 83])
        dpg.add_text("Rev", tag="TrimH_R_Rev", pos=[trim_window_left_block-77,  83])
        dpg.add_text("Right Side Servo Values", tag="TrimH_R_Title", pos=[
                     trim_window_left_block-30, 83])

        # --- 右サーボ列 (R0-R14) ----------------------------------------------------
        for i in range(MRD_SERVO_SLOTS):
            base_y = 108 + i * 25
            slider_tag = f"Trim_R{i}"

            dpg.add_text(f"R{i}", tag=f"TrimLabel_R{i}", pos=[trim_window_left_block-162, base_y])
            dpg.add_checkbox(tag=f"Mount_R{i}", default_value=mrd.servo_mount[f"R{i}"],
                             callback=toggle_servo_mount, user_data=f"R{i}", pos=[trim_window_left_block-135, base_y])
            dpg.add_input_text(tag=f"ID_R{i}", default_value=str(mrd.servo_id_values[f"R{i}"]),
                               width=25, callback=set_servo_id, user_data=f"R{i}", pos=[trim_window_left_block-107, base_y])
            dpg.add_checkbox(tag=f"Direction_R{i}", default_value=mrd.servo_direction[f"R{i}"],
                             callback=toggle_servo_direction, user_data=f"R{i}", pos=[trim_window_left_block-75, base_y])

            # スライダー
            dpg.add_slider_float(default_value=0.0, tag=slider_tag, max_value=180, min_value=-180, width=100,  # label=f"R{i}",
                                 callback=set_servo_angle_from_trim, pos=[trim_window_left_block-50, base_y])

            # -/+ ボタン
            dpg.add_button(label="-", tag=f"TrimMinus_R{i}", width=18, pos=[
                           trim_window_left_block+55, base_y], callback=step_trim, user_data=(slider_tag, -1))
            dpg.add_button(label="+", tag=f"TrimPlus_R{i}", width=18, pos=[
                           trim_window_left_block+77, base_y], callback=step_trim, user_data=(slider_tag, +1))

            # インプットフィールド
            dpg.add_input_text(tag=f"Input_Trim_R{i}", decimal=True, width=50, pos=[
                               trim_window_left_block+100, base_y])

            # エンターボタン
            dpg.add_button(label="Enter", tag=f"Enter_Trim_R{i}", callback=apply_trim_input_value, user_data=f"R{i}",
                           width=42, pos=[trim_window_left_block+155, base_y])

        # --- ヘッダー(左サーボ列) -------------------------------------------------
        dpg.add_text("Idx", tag="TrimH_L_Idx", pos=[trim_window_right_block-165, 83])
        dpg.add_text("Mt",  tag="TrimH_L_Mt",  pos=[trim_window_right_block-132, 83])
        dpg.add_text("ID",  tag="TrimH_L_ID",  pos=[trim_window_right_block-104, 83])
        dpg.add_text("Rev", tag="TrimH_L_Rev", pos=[trim_window_right_block-77,  83])
        dpg.add_text("Left Side Servo Values", tag="TrimH_L_Title", pos=[
                     trim_window_right_block-30, 83])

        # --- 左サーボ列 (L0-L14) ----------------------------------------------------
        for i in range(MRD_SERVO_SLOTS):
            base_y = 108 + i * 25
            slider_tag = f"Trim_L{i}"

            dpg.add_text(f"L{i}", tag=f"TrimLabel_L{i}", pos=[trim_window_right_block-162, base_y])
            dpg.add_checkbox(tag=f"Mount_L{i}", default_value=mrd.servo_mount[f"L{i}"],
                             callback=toggle_servo_mount, user_data=f"L{i}", pos=[trim_window_right_block-135, base_y])
            dpg.add_input_text(tag=f"ID_L{i}", default_value=str(mrd.servo_id_values[f"L{i}"]),
                               width=25, callback=set_servo_id, user_data=f"L{i}", pos=[trim_window_right_block-107, base_y])
            dpg.add_checkbox(tag=f"Direction_L{i}", default_value=mrd.servo_direction[f"L{i}"],
                             callback=toggle_servo_direction, user_data=f"L{i}", pos=[trim_window_right_block-75, base_y])

            # スライダー
            dpg.add_slider_float(default_value=0.0, tag=slider_tag, max_value=180, min_value=-180, width=100,  # label=f"L{i}",
                                 callback=set_servo_angle_from_trim, pos=[trim_window_right_block-50, base_y])

            # -/+ ボタン
            dpg.add_button(label="-", tag=f"TrimMinus_L{i}", width=18, pos=[
                           trim_window_right_block+55, base_y], callback=step_trim, user_data=(slider_tag, -1))
            dpg.add_button(label="+", tag=f"TrimPlus_L{i}", width=18, pos=[
                           trim_window_right_block+77, base_y], callback=step_trim, user_data=(slider_tag, +1))

            # インプットフィールド
            dpg.add_input_text(tag=f"Input_Trim_L{i}", decimal=True, width=50, pos=[
                               trim_window_right_block+100, base_y])

            # エンターボタン
            dpg.add_button(label="Enter", tag=f"Enter_Trim_L{i}", callback=apply_trim_input_value, user_data=f"L{i}",
                           width=42, pos=[trim_window_right_block+155, base_y])

        # --- 下部ボタン行 (Power / Enable / Raw Zero / Trim Zero / Export Settings / Show Setting Data) ---
        # right-anchored, 20px margin, 8px gaps: Show(140) | Export(125) | TrimZero(80) | RawZero(70)
        dpg.add_button(label="Raw Zero", callback=send_raw_zero_temp,
                       pos=[viewport_width-479, viewport_height-52], width=70)
        dpg.add_button(label="Trim Zero", callback=send_trim_zero_restore,
                       pos=[viewport_width-401, viewport_height-52], width=80)
        dpg.add_button(label="Export Settings", callback=export_settings_to_file,
                       pos=[viewport_width-313, viewport_height-52], width=125)
        dpg.add_button(label="Show Setting Data", callback=show_setting_data_modal,
                       pos=[viewport_width-180, viewport_height-52], width=140)

        # --- グレーアウト用オーバーレイ(ウィジェット追加後に描画→最前面) ----------
        # drawlist は DPG 2.x で pos が効かないため child_window を使用
        # [1] サーボエリア: ヘッダー(y=73)からサーボ行末尾まで
        _cover_y = 73
        _cover_h = (viewport_height - 90) - _cover_y
        _cover_w = viewport_width - 40
        with dpg.child_window(tag="TrimCoverOverlay", width=_cover_w, height=_cover_h,
                              pos=[0, _cover_y], no_scrollbar=True, border=False):
            pass
        # [2] ステップツールバー中段(Step2/Step3エリア): Step1とExitの間を塗りつぶす
        with dpg.child_window(tag="TrimCoverToolbarMid", width=viewport_width - 340, height=39,
                              pos=[200, 26], no_scrollbar=True, border=False):
            pass
        # [3] 下段行全体(Power/Enable/ボタン群) + deltaフィールド(viewport_height-92)まで覆う
        with dpg.child_window(tag="TrimCoverBottomRow", width=viewport_width, height=100,
                              pos=[0, viewport_height - 100], no_scrollbar=True, border=False):
            pass
        # ウィンドウ背景色と同色テーマを適用
        with dpg.theme() as _cover_theme:
            with dpg.theme_component(dpg.mvChildWindow):
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, [37, 37, 38, 255])
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 0, 0)
        dpg.bind_item_theme("TrimCoverOverlay",    _cover_theme)
        dpg.bind_item_theme("TrimCoverToolbarMid", _cover_theme)
        dpg.bind_item_theme("TrimCoverBottomRow",  _cover_theme)

        # ステップハイライト用drawlist: overlaysより後に追加して最前面に描画
        # draw_rectangleはマウスイベントを取らないためボタンは引き続き操作可能
        # HL1/HL2右端: Step3ボタン右端+5px、ただしExitボタン左端-5pxでキャップ
        _hl_step3_right = min(_s * 2 + 218, viewport_width - 145)
        with dpg.drawlist(tag="TrimStepDrawlist", width=viewport_width, height=38, pos=[0, 26]):
            dpg.draw_rectangle([0, 2], [205, 34], tag="TrimStepHL0",
                               color=[100, 180, 255, 220], fill=[0, 0, 0, 0],
                               thickness=2, rounding=4)
            dpg.draw_rectangle([_s + 5, 2], [_hl_step3_right, 34], tag="TrimStepHL1",
                               color=[0, 0, 0, 0], fill=[0, 0, 0, 0],
                               thickness=2, rounding=4)
            dpg.draw_rectangle([_s + 5, 2], [_hl_step3_right, 34], tag="TrimStepHL2",
                               color=[0, 0, 0, 0], fill=[0, 0, 0, 0],
                               thickness=2, rounding=4)
            dpg.draw_rectangle([viewport_width - 158, 2], [viewport_width - 35, 34], tag="TrimStepHL3",
                               color=[0, 0, 0, 0], fill=[0, 0, 0, 0],
                               thickness=2, rounding=4)

        # Start Trim Setting を押すまで行エリアをオーバーレイで隠す
        set_trim_area_enabled(False)


UDP_SEND_IP_DEF, UDP_RECV_IP_DEF, NETWORK_MODE = select_network_mode_and_ip()
UDP_SEND_IP = UDP_SEND_IP_DEF

# ================================================================================================================
# ---- メインループ ------------------------------------------------------------------------------------------------
# ================================================================================================================

# ------------------------------------------------------------------------
# [ 0 ]  初期設定
# ------------------------------------------------------------------------
mrd = MeridianConsole()  # Meridianデータのインスタンス
redis_receiver = RedisReceiver(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_READ)
redis_transfer = RedisTransfer(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_WRITE)


def fetch_redis_data():
    """<- Redis: meridis_calc_pub からhash形式でデータを受信してs_meridimに反映する。"""
    if not mrd.flag_redis_sub:
        return
    if mrd.flag_ros1_sub:
        return
    data = redis_receiver.get_data()
    if data is None or len(data) != 90:
        return
    for i in range(21, 81, 2):
        mrd.s_meridim[i] = int(data[i] * 100)


def send_redis_data():
    """-> Redis: Axis MonitorのTarget/Actualに連動してs_meridim/r_meridimをchecksum付きで書き込む。"""
    if not mrd.flag_redis_pub:
        return
    src = mrd.s_meridim if mrd.flag_display_mode else mrd.r_meridim
    s_int16 = np.array(src[:MSG_SIZE-1], dtype=np.int16)
    checksum = np.int16(~np.sum(s_int16, dtype=np.int16))
    data = [float(v) / 100.0 for v in src[:MSG_SIZE-1]]
    data.append(float(checksum))
    redis_transfer.set_data(data)


def meridian_loop():
    import sys
    sys.setswitchinterval(0.001)  # GILスイッチ間隔を1msに短縮しGIL待ち時間を削減
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP用のsocket設定
    try:
        sock.bind(('', UDP_RECV_PORT))  # 全インターフェースで受信
    except OSError as e:
        if e.errno == 48:  # Address already in use
            mrd.message1 = "Address already in use"
            mrd.flag_self_mode = True  # DPGループが次フレームでチェックボックスに反映する
        else:
            raise
    sock.settimeout(0.1)  # Selfモード切替時にrecvfromのブロックを解除するためのタイムアウト

    _checksum = np.array([0], dtype=np.int16)
    atexit.register(cleanup)  # この行は機能しているかどうかわからない

    while mrd.running:
        print("Start.")
        # 180個の要素を持つint8型のNumPy配列を作成
        _r_bin_data_past = np.zeros(180, dtype=np.int8)
        _r_bin_data = np.zeros(180, dtype=np.int8)
        mrd.message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."

        with closing(sock):
            _prev_self_mode = False  # Self→UDP切り替え検出用
            _self_next_frame_time = 0.0  # Selfモード絶対タイマー(累積ドリフト補正用)
            while mrd.running:
                mrd.loop_count += 1  # このpythonを起動してからのフレーム数をカウントアップ
                _loop_start = time.time()

# ------------------------------------------------------------------------
# [ 1 ] : UDPデータの受信
# ------------------------------------------------------------------------
                if mrd.flag_self_mode:
# [ 1-Self ] : Selfモード: UDP受信を無視し s_meridim を r_meridim にコピーして 100Hz 動作
                    if not _prev_self_mode:
                        # Self mode 開始: 絶対フレームタイマーを現在時刻から初期化
                        _self_next_frame_time = _loop_start + 0.01
                    # [4-1]のチェックサム検証を通過させるためコピー前にチェックサムを計算
                    mrd.s_meridim[MSG_SIZE-1] = np.int16(~np.sum(mrd.s_meridim[:MSG_SIZE-1], dtype=np.int16))
                    _self_bin = mrd.s_meridim.tobytes()  # tobytes()でnumpy→bytes直接変換(structより高速)
                    mrd.r_meridim = struct.unpack('90h', _self_bin)
                    mrd.r_meridim_ushort = struct.unpack('90H', _self_bin)
                    mrd.r_meridim_char = struct.unpack('180b', _self_bin)
                    mrd.message1 = "Self mode: 100Hz (no UDP)"
                    _prev_self_mode = True
                else:
                    # Self→UDP切り替え時: _r_bin_dataをリセットして差分検出を確実にする
                    if _prev_self_mode:
                        _r_bin_data_past = np.zeros(180, dtype=np.int8)
                        _r_bin_data = np.zeros(180, dtype=np.int8)
                    _prev_self_mode = False
# [ 1-1 ] : UDPデータの受信を待つループ
                    try:
                        _r_bin_data_past = _r_bin_data
                        _r_bin_data, addr = sock.recvfrom(MSG_BUFF)  # UDPに受信したデータを転記
                        while np.array_equal(_r_bin_data_past, _r_bin_data):  # 前回受信データと差分があったら進む
                            _r_bin_data, addr = sock.recvfrom(MSG_BUFF)
                    except socket.timeout:
                        continue  # タイムアウト時はループ先頭に戻りflag_self_modeを再チェック

# [ 1-2 ] : 受信UDPデータの変換
                    # 受信データをshort型のMeridim90に変換
                    mrd.r_meridim = struct.unpack('90h', _r_bin_data)
                    mrd.r_meridim_ushort = struct.unpack(
                        '90H', _r_bin_data)  # unsignedshort型
                    mrd.r_meridim_char = struct.unpack('180b', _r_bin_data)
                    mrd.message1 = "UDP data receiving from "+UDP_SEND_IP  # 受信中のメッセージ表示

# [ 1-3 ] : 送信UDPデータのターミナル表示
                if mrd.flag_disp_send:
                    mrd.flag_disp_send = 1
                    print('send:'+' '.join(map(str, mrd.s_meridim)))

# [ 1-4 ] : 受信UDPデータのターミナル表示
                if mrd.flag_disp_rcvd:
                    mrd.flag_disp_rcvd = 1
                    print('rcvd:'+' '.join(map(str, mrd.r_meridim)))

# ------------------------------------------------------------------------
# [ 2 ] : 受信データのチェック
# ------------------------------------------------------------------------
# [ 2-1 ] : チェックサムの確認
                # 受信データに対するチェックサム値の計算
                _checksum[0] = np.int16(
                    ~np.sum(mrd.r_meridim[:MSG_SIZE-1], dtype=np.int16))
                _temp_int16 = np.int16(0)  # エラーフラグのカウント用変数

                if _checksum[0] != mrd.r_meridim[MSG_SIZE-1]:  # チェックサムがNGの処理
                    # エラーフラグ15ビット目:PCのUDP受信エラーフラグを上げる
                    _temp_int16 = (
                        mrd.r_meridim[MSG_ERRS] & 0xFFFF) | 0b1000000000000000
                    mrd.error_count_esp_to_pc += 1  # PCのUDP受信エラーをカウントアップ

# [ 2-2 ] : チェックサムOKデータのエラーフラグ処理
                else:
                    # エラーフラグを調べる
                    # 14ビット目:ESP32のPCからのUDP受信のエラー
                    if (mrd.r_meridim[MSG_ERRS] >> 14 & 1) == 1:
                        mrd.error_count_pc_to_esp += 1
                    # 13ビット目:TeensyのESP32からのSPI受信のエラー
                    if (mrd.r_meridim[MSG_ERRS] >> 13 & 1) == 1:
                        mrd.error_count_esp_to_tsy += 1
                    # 12ビット目:ESPのTeensyからのSPI受信のエラー
                    if (mrd.r_meridim[MSG_ERRS] >> 12 & 1) == 1:
                        mrd.error_count_tsy_to_esp += 1
                    # 11ビット目:Teensyのシステムディレイのエラー
                    if (mrd.r_meridim[MSG_ERRS] >> 11 & 1) == 1:
                        mrd.error_count_tsy_delay += 1
                    # 10ビット目:ESPのPCからのUDP受信のフレーム連番スキップフラグ
                    if (mrd.r_meridim[MSG_ERRS] >> 10 & 1) == 1:
                        mrd.error_count_esp_skip += 1
                    # 9ビット目:TeensyのESP経由のPCから受信のフレーム連番スキップフラグ
                    if (mrd.r_meridim[MSG_ERRS] >> 9 & 1) == 1:
                        mrd.error_count_tsy_skip += 1
                    # サーボ値の受信に失敗したサーボID(エラーフラグ下位8ビット)を調べる
                    _temp_int16 = mrd.r_meridim[MSG_ERRS] & 0b0000000011111111
                    mrd.error_servo_id_past = mrd.error_servo_id
                    if _temp_int16 > 0:
                        mrd.error_count_servo_skip += 1
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

# [ 2-3 ] : 断線エラーのサーボID表示処理
                    if (mrd.error_servo_id_past != mrd.error_servo_id):
                        if mrd.error_servo_id == "None":
                            print("Servo error gone...")
                        else:
                            print("Found servo error: "+mrd.error_servo_id)

# [ 2-4 ] : 末端処理
                    # エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を下げる
                    _temp_int16 = mrd.r_meridim[MSG_ERRS] & 0b0111111111111111
                    # フレームスキップチェック用のカウントの代入
                    mrd.frame_sync_r_recv = mrd.r_meridim_ushort[1]

# ------------------------------------------------------------------------
# [ 3 ] : ステップモードの場合はここでループ待機
# ------------------------------------------------------------------------
                if mrd.flag_set_flow_or_step < 0:
                    while mrd.flag_stop_flow:
                        if mrd.flag_allow_flow:
                            break
                        time.sleep(0.005)  # CPUの負荷を下げる
                    time.sleep(0.001)
                    mrd.flag_stop_flow = True

# ------------------------------------------------------------------------
# [ 4 ] : チェックOKの受信UDPデータについての処理
# ------------------------------------------------------------------------
# [ 4-1 ] : チェックサムがOK かつ (Selfモードまたはシーケンス番号が前回と異なっていれば), 処理に回す
                if (_checksum[0] == mrd.r_meridim[MSG_SIZE-1]) and (mrd.flag_self_mode or mrd.frame_sync_r_recv != mrd.frame_sync_r_last):

                    # マスターコマンドがMSG_SIZEより大きければ, 特殊コマンドを実行
                    if (mrd.r_meridim[MRD_MASTER] > MSG_SIZE):

                        if mrd.r_meridim[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA0:
                            print('rcvd EEPROM[0][*]:' +
                                  ' '.join(map(str, mrd.r_meridim)))

                        if mrd.r_meridim[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA1:
                            print('rcvd EEPROM[1][*]:' +
                                  ' '.join(map(str, mrd.r_meridim)))
                            # EEPROMからのデータを処理してチェックボックスに反映
                            process_eeprom_data()

                        if mrd.r_meridim[MRD_MASTER] == MCMD_EEPROM_BOARDTOPC_DATA2:
                            print('rcvd EEPROM[2][*]:' +
                                  ' '.join(map(str, mrd.r_meridim)))

                        mrd.s_meridim[MRD_MASTER] = 90

                    # 以降は特殊コマンドではなく, 通常フローの実行
                    else:
                        # 受信データを送信データに転記
                        mrd.s_meridim[:MSG_SIZE-1] = mrd.r_meridim[:MSG_SIZE-1]

    # [ 4-2 ] : シーケンス番号の処理
                        mrd.frame_sync_r_expect += 1  # 予想シーケンス番号のカウントアップ
                        if mrd.frame_sync_r_expect > 59999:
                            mrd.frame_sync_r_expect = 0

                        if (mrd.frame_sync_r_recv == mrd.frame_sync_r_expect):  # 受信したカウントが予想通りであればスキップなし
                            _temp_int16 &= 0b1111111011111111  # PCのESP経由Teensyからの連番スキップフラグを下げる
                        else:
                            _temp_int16 |= 0b0000000100000000  # PCのESP経由Teensyからの連番スキップフラグを上げる
                            mrd.frame_sync_r_expect = mrd.frame_sync_r_recv  # 受信カウントの方が多ければズレを検出し, 追いつく
                            mrd.error_count_pc_skip += 1  # スキップカウントをプラス

    # [ 4-3 ] : 最終サーボ位置情報のキープ
                        if mrd.flag_servo_power == 2:  # サーボオンボタン押下初回のみ最終受け取りサーボ情報をキープ
                            for i in range(21, 81, 2):
                                mrd.s_meridim_motion_keep_f[i] = mrd.r_meridim[i]*0.01
                            mrd.flag_servo_power = 1

                        if mrd.flag_servo_power == -1:  # サーボオフボタン押下初回のみ最終送信サーボ情報をキープ
                            for i in range(21, 81, 2):
                                mrd.s_meridim_motion_keep_f[i] = mrd.s_meridim[i]*0.01
                            mrd.flag_servo_power = 0

    # ------------------------------------------------------------------------
    # [ 5 ] : 送信用UDPデータの作成
    # ------------------------------------------------------------------------
    # [ 5-1 ] : 送信するサーボ位置を以下の場合別に s_meridim_motion に格納
                        if _checksum[0] == mrd.r_meridim[MSG_SIZE-1]:  # 受信成功時はデータ更新
                            mrd.s_meridim.fill(0)  # 配列内のデータをゼロでリセット

    # ▶︎ 5-1-1 : ① 受信値そのままの場合：送信データのベースを受信データのコピーで作成
                        # サーボパワーオン時は, 電源入力時に保持した値を固定で流す(ハウリング的なサーボ位置ズレの増幅を防止)
                        if mrd.flag_servo_power:
                            for i in range(21, 81, 2):
                                mrd.s_meridim[i] = int(
                                    mrd.s_meridim_motion_keep_f[i]*100)
                        else:
                            if mrd.flag_resv_data:
                                for i in range(21, 81, 2):  # 受信サーボ値を書き込みモーションのベースとして一旦キープ
                                    mrd.s_meridim_motion_f[i] = mrd.r_meridim[i]*0.01

    # ▶︎ 5-1-2 : ② サーボ位置にROSのサブスクライブを反映させる場合にはここでデータを作成★★
                        if mrd.flag_ros1_sub:
                            for i in range(MRD_SERVO_SLOTS):
                                mrd.s_meridim_motion_f[MRD_L_ORIG_IDX + 1 + i *
                                                       2] = mrd.s_meridim_js_sub_f[MRD_L_ORIG_IDX + 1 + i * 2]
                                mrd.s_meridim_motion_f[MRD_R_ORIG_IDX + 1 + i *
                                                       2] = mrd.s_meridim_js_sub_f[MRD_R_ORIG_IDX + 1 + i * 2]

    # ▶︎ 5-1-3 : ③ サーボ位置をここで計算制御する場合は以下でデータを作成(まずはデモモーションのみで運用テスト)
                        if mrd.flag_demo_action:
                            # xをフレームごとにカウントアップ
                            mrd.x += math.pi/STEP
                            if mrd.x > math.pi*2000:
                                mrd.x = 0

                            # サインカーブで全身をくねらせる様にダンス
                            mrd.s_meridim_motion_f[21] = int(np.sin(mrd.x)*30)          # 頭ヨー
                            mrd.s_meridim_motion_f[23] = int(np.sin(mrd.x)*10) + 20     # 左肩ピッチ
                            mrd.s_meridim_motion_f[25] = - int(np.sin(mrd.x*2)*10) + 10 # 左肩ロール
                            mrd.s_meridim_motion_f[27] = int(np.sin(mrd.x)*10) + 10     # 左肘ヨー
                            mrd.s_meridim_motion_f[29] = int(np.sin(mrd.x)*30) - 30     # 左肘ピッチ
                            mrd.s_meridim_motion_f[31] = int(np.sin(mrd.x)*5)           # 左股ヨー
                            mrd.s_meridim_motion_f[33] = - int(np.sin(mrd.x)*4)         # 左股ロール
                            mrd.s_meridim_motion_f[35] = int(np.sin(mrd.x*2)*20) - 2    # 左股ピッチ
                            mrd.s_meridim_motion_f[37] = - int(np.sin(mrd.x*2)*40)      # 左膝ピッチ
                            mrd.s_meridim_motion_f[39] = int(np.sin(mrd.x*2)*20) - 2    # 左足首ピッチ
                            mrd.s_meridim_motion_f[41] = int(np.sin(mrd.x)*4)           # 左足首ロール
                            mrd.s_meridim_motion_f[51] = - int(np.sin(mrd.x)*20)        # 腰ヨー
                            mrd.s_meridim_motion_f[53] = - int(np.sin(mrd.x)*10) + 20   # 右肩ピッチ
                            mrd.s_meridim_motion_f[55] = - int(np.sin(mrd.x*2)*10) + 10 # 右肩ロール
                            mrd.s_meridim_motion_f[57] = - int(np.sin(mrd.x)*10) + 10   # 右肘ヨー
                            mrd.s_meridim_motion_f[59] = - int(np.sin(mrd.x)*30) - 30   # 右肘ピッチ
                            mrd.s_meridim_motion_f[61] = - int(np.sin(mrd.x)*5)         # 右股ヨー
                            mrd.s_meridim_motion_f[63] = int(np.sin(mrd.x)*4)           # 右股ロール
                            mrd.s_meridim_motion_f[65] = - int(np.sin(mrd.x*2)*20) - 2  # 右股ピッチ
                            mrd.s_meridim_motion_f[67] = int(np.sin(mrd.x*2)*40)        # 右膝ピッチ
                            mrd.s_meridim_motion_f[69] = - int(np.sin(mrd.x*2)*20) - 2  # 右足首ピッチ
                            # 右足首ロール
                            mrd.s_meridim_motion_f[71] = -int(np.sin(mrd.x)*4)
                            if mrd.flag_enable_send_made_data:
                                for i in range(MRD_SERVO_SLOTS):
                                    mrd.s_meridim_motion_keep_f[MRD_L_ORIG_IDX + 1 + i *
                                                                2] = mrd.s_meridim_motion_f[MRD_L_ORIG_IDX + 1 + i * 2]
                                    mrd.s_meridim_motion_keep_f[MRD_R_ORIG_IDX + 1 + i *
                                                                2] = mrd.s_meridim_motion_f[MRD_R_ORIG_IDX + 1 + i * 2]

    # ▶︎ 5-1-4 : ④ ユーザーがサーボ位置処理を反映させる場合 → ここで自由にコードを作成
                        # redisからのデータを仮にここで処理
                        fetch_redis_data()


    # [ 5-2 ] : [Zero]ボタン押下時に全サーボ位置=0を送信(Board trim適用後 → Trim Zero)
                        if mrd.flag_servo_zero > 0:
                            for i in range(MRD_SERVO_SLOTS):
                                mrd.s_meridim[MRD_L_ORIG_IDX + 1 + i * 2] = 0
                                mrd.s_meridim[MRD_R_ORIG_IDX + 1 + i * 2] = 0
                                mrd.s_meridim_motion_f[MRD_L_ORIG_IDX + 1 + i * 2] = 0
                                mrd.s_meridim_motion_f[MRD_R_ORIG_IDX + 1 + i * 2] = 0
                                mrd.s_meridim_motion_keep_f[MRD_L_ORIG_IDX + 1 + i * 2] = 0
                                mrd.s_meridim_motion_keep_f[MRD_R_ORIG_IDX + 1 + i * 2] = 0
                            mrd.flag_servo_zero = 0

    # [ 5-3 ] : PC側発行のサーボ位置をs_meridimに書き込む
                        if mrd.flag_enable_send_made_data:  # PC側発行データの送信Enable判定
                            for i in range(21, 81, 2):
                                if mrd.flag_demo_action | mrd.flag_python_action | mrd.flag_enable_send_made_data | mrd.flag_ros1_sub:
                                    mrd.s_meridim[i] = int(mrd.s_meridim_motion_f[i]*100)
                                else:  # Consoleでモーションを指定しない場合はハンチング防止としてサーボオフ時のデータを送信
                                    mrd.s_meridim[i] = int(mrd.s_meridim_motion_keep_f[i]*100)

    # [ 5-4 ] : サーボオンオフフラグチェック：サーボオンフラグを格納
                        mrd.s_meridim[20:80:2] = 1 if mrd.flag_servo_power > 0 else 0

    # [ 5-5 ] : リモコンデータをリセットし, PCからのリモコン入力値を格納
                        # temp = np.array([0], dtype=np.uint16)  # uint16に変更
                        # temp[0] = 0
                        # temp[0] = mrd.pad_button_panel_short[0]
                        # mrd.s_meridim[15] = int(temp[0])  # 安全に変換して格納
                        # pad_button_tmp = np.int16(mrd.pad_button_panel_short[0])
                        # mrd.s_meridim[15] = pad_button_tmp  # ボタン
                        # pad_button_tmp = mrd.pad_button_panel_short[0] & 0xFFFF  # uint16保証
                        # mrd.s_meridim[15] = np.int16(pad_button_tmp)  # int16に変換して格納

                        pad_button_tmp = np.uint16(
                            mrd.pad_button_panel_short[0] | mrd.pad_gamepad_buttons[0])
                        mrd.s_meridim[15] = np.int16(pad_button_tmp)  # uint16→int16変換

                        if mrd.pad_gamepad_connected:
                            axes = mrd.pad_gamepad_axes
                            lx  = int(np.clip(axes[0] * 127, -127, 127))
                            ly  = int(np.clip(axes[1] * 127, -127, 127))
                            rx  = int(np.clip(axes[2] * 127, -127, 127))
                            ry  = int(np.clip(axes[3] * 127, -127, 127))
                            l2v = int(np.clip((axes[4] + 1.0) / 2.0 * 255, 0, 255))
                            r2v = int(np.clip((axes[5] + 1.0) / 2.0 * 255, 0, 255))
                            mrd.s_meridim[16] = struct.unpack('<h', bytes([ly & 0xFF, lx & 0xFF]))[0]
                            mrd.s_meridim[17] = struct.unpack('<h', bytes([ry & 0xFF, rx & 0xFF]))[0]
                            mrd.s_meridim[18] = struct.unpack('<h', bytes([l2v, r2v]))[0]
                        else:
                            mrd.s_meridim[16] = 0  # アナログ1
                            mrd.s_meridim[17] = 0  # アナログ2
                            mrd.s_meridim[18] = 0  # アナログ3

    # [ 5-6 ] : 送信マスターコマンドの作成
                        mrd.s_meridim[0] = MSG_SIZE  # デフォルト値を格納

    # ▶︎ 5-6-1 : ヨー軸センターリセットコマンドを格納
                        if (mrd.flag_update_yaw > 0):
                            mrd.flag_update_yaw -= 1
                            mrd.s_meridim[0] = MCMD_SENSOR_YAW_CALIB
                            if (mrd.flag_update_yaw == 0):
                                print(
                                    "Send COMMAND : caliblate sensor's yaw.:["+str(MCMD_SENSOR_YAW_CALIB)+"]")

    # ▶︎ 5-6-2 : フローモード(ボード側が周期制御を持つ)への切り替え
                        if mrd.flag_set_flow_or_step == 2:
                            mrd.s_meridim[0] = MCMD_BOARD_TRANSMIT_ACTIVE
                            mrd.flag_set_flow_or_step = 1

    # ▶︎ 5-6-3 : ステップモード(PC側が周期制御を持つ)への切り替え
                        if mrd.flag_set_flow_or_step == -2:
                            mrd.s_meridim[0] = MCMD_BOARD_TRANSMIT_PASSIVE
                            mrd.flag_set_flow_or_step = -1

    # ▶︎ 5-6-4 : ボード側のフレーム管理時計を現在時間でリセット
                        if mrd.frag_reset_cycle:
                            mrd.s_meridim[0] = MCMD_FRAMETIMER_RESET
                            mrd.frag_reset_cycle = False

    # [ 5-7 ] : エラーフラグの処理
    # ▶︎ 5-7-1 : Meridim内のエラーフラグをリセット
                        if mrd.frag_reset_errors:
                            mrd.s_meridim[MSG_ERRS] = 0
                            mrd.error_servo_id = "None"     # 受信エラーのあったサーボのIDを格納
                            mrd.error_servo_id_past = "None"     # 受信エラーのあったサーボのIDを格納
                            mrd.frag_reset_errors = False

    # ▶︎ 5-7-2 : キープしたエラーフラグを格納
                        mrd.s_meridim[MSG_ERRS] = _temp_int16

    # [ 5-8 ] : 送信用シーケンス番号の作成と格納
                        mrd.frame_sync_s += 1  # 送信用のframe_sync_sをカウントアップ
                        if mrd.frame_sync_s > 59999:  # 60,000以上ならゼロリセット
                            mrd.frame_sync_s = 0
                        if mrd.frame_sync_s > 32767:  # unsigned short として取り出せるようなsinged shortに変換
                            mrd.s_meridim[1] = mrd.frame_sync_s-65536
                        else:
                            mrd.s_meridim[1] = mrd.frame_sync_s  # & 0xffff

    # [ 5-9 ] : ミニターミナルからのデータ送信処理
                        if mrd.flag_terminal_mode_send > 0:  # ミニターミナルの送信モードの確認
                            print_string = ""
                            for i in range(8):
                                if ((mrd.s_miniterminal_keep[i][0] >= 0) and (mrd.s_miniterminal_keep[i][0] < MSG_SIZE)):
                                    mrd.s_meridim[int(mrd.s_miniterminal_keep[i][0])] = int(
                                        mrd.s_miniterminal_keep[i][1])
                                    print_string = print_string + \
                                        "["+str(int(mrd.s_miniterminal_keep[i][0]))+"] " + \
                                        str(int(mrd.s_miniterminal_keep[i][1]))+", "
                                    # サーボパワーオン時のキープ配列にも反映しておく. こうするとミニターミナルから脱力してサーボを回転させた後にサーボパワーオンで位置の固定ができる
                                    mrd.s_meridim_motion_keep_f[int(mrd.s_miniterminal_keep[i][0])] = int(
                                        mrd.s_miniterminal_keep[i][1]*0.01)

                            if mrd.flag_terminal_mode_send == 2:  # 送信データを一回表示
                                mrd.flag_terminal_mode_send = 1

                            if mrd.flag_send_miniterminal_data_once == 1:    # ミニターミナルの値を1回送信する
                                mrd.flag_terminal_mode_send = 0
                                mrd.flag_send_miniterminal_data_once = 0

# [ 5-10 ] : 特殊コマンドのデータ送信処理
                    # 特殊コマンド送信モードの判定 [6-2]で完了処理
                    if mrd.flag_special_command_send > 0:
                        for i in range(MSG_SIZE):
                            mrd.s_meridim[i] = mrd.s_meridim_special[i]

                        # 送信用シーケンス番号の作成と格納
                        if mrd.frame_sync_s > 59999:  # 60,000以上ならゼロリセット
                            mrd.frame_sync_s = 0
                        if mrd.frame_sync_s > 32767:  # unsigned short として取り出せるようなsinged shortに変換
                            mrd.s_meridim[1] = mrd.frame_sync_s-65536
                        else:
                            mrd.s_meridim[1] = mrd.frame_sync_s  # & 0xffff

                        # 特殊コマンドを送信するのでログに出力
                        print(
                            f"Sending special command: {mrd.s_meridim[MRD_MASTER]}")

# [ 5-11 ] : 格納した送信データについてチェックサムを追加
                    s_meridim_int16 = np.array(
                        mrd.s_meridim[:MSG_SIZE-1], dtype=np.int16)
                    _checksum[0] = np.int16(
                        ~np.sum(s_meridim_int16, dtype=np.int16))
                    mrd.s_meridim[MSG_SIZE-1] = _checksum[0]

# ------------------------------------------------------------------------
# [ 6 ] : UDPデータを送信
# ------------------------------------------------------------------------
    # [ 6-1 ] : UDPデータの送信処理
                    s_bin_data = struct.pack(
                        '90h', *mrd.s_meridim)       # データをパック
                    if not mrd.flag_self_mode:
                        sock.sendto(s_bin_data, (UDP_SEND_IP,
                                    UDP_SEND_PORT))  # UDP送信
                    send_redis_data()              # -> Redis: s_meridimをRedisに書き込む
                    now = time.time()-mrd.start+0.0001
    # [ 6-2 ] : 特殊コマンドのデータ送信の完了処理
                    if mrd.flag_special_command_send > 0:
                        _last_special_cmd = int(mrd.s_meridim_special[MRD_MASTER])
                        mrd.s_meridim_special = np.zeros(
                            MSG_SIZE, dtype=np.int16)  # 特殊コマンド用のデータをクリア
                        mrd.flag_special_command_send = 0  # 特殊コマンドの送信フラグを下げる
                        # SAVE_TRIM直後はStart Trim Settingと同じコマンドを続けて送信
                        # → EEPROMから保存済みtrimを再ロードしてTrim Zeroへ移動, trimモードに戻る
                        if _last_special_cmd == MCMD_EEPROM_SAVE_TRIM:
                            start_trim_setting()

    # ------------------------------------------------------------------------
    # [ 7 ] : 表示処理
    # ------------------------------------------------------------------------
                    # マスターコマンドがMSG_SIZEより大きければ, 特殊コマンドの実行として下記の表示更新をキャンセル
                    if (mrd.r_meridim[MRD_MASTER] <= MSG_SIZE):

                        # [ 7-1 ] : Axis monitor の表示データ切り替え
                        # 1=target data(send data),0= actual data(received data)
                        if mrd.flag_display_mode:
                            mrd.d_meridim[:MSG_SIZE-1] = mrd.s_meridim[:MSG_SIZE-1]
                        else:
                            mrd.d_meridim[:MSG_SIZE-1] = mrd.r_meridim[:MSG_SIZE-1]

    # [ 7-2 ] : メッセージウィンドウの表示更新(3フレームに1回)
                        if mrd.loop_count % 3 == 0:
                            mrd.message2 = "ERROR COUNT ESP-PC:"+str("{:}".format(mrd.error_count_esp_to_pc)) + " PC-ESP:"+str("{:}".format(mrd.error_count_pc_to_esp))+" ESP-TSY:"+str(
                                "{:}".format(mrd.error_count_esp_to_tsy)) + " TSY_Delay:"+str("{:}".format(mrd.error_count_tsy_delay)) + "    Servo_trouble:"+mrd.error_servo_id

                            mrd.message3 = "ERROR RATE ESP-PC:"+str("{:.2%}".format(mrd.error_count_esp_to_pc/mrd.loop_count)) + " PC-ESP:"+str("{:.2%}".format(mrd.error_count_pc_to_esp/mrd.loop_count))+" ESP-TSY:"+str("{:.2%}".format(
                                mrd.error_count_esp_to_tsy/mrd.loop_count)) + " TsySKIP:"+str("{:.2%}".format(mrd.error_count_tsy_skip/mrd.loop_count)) + " ESPSKIP:" + str("{:.2%}".format(mrd.error_count_esp_skip/mrd.loop_count))

                            mrd.message4 = "SKIP COUNT Tsy:" + str("{:}".format(mrd.error_count_tsy_skip))+" ESP:"+str("{:}".format(mrd.error_count_esp_skip))+" PC:"+str("{:}".format(mrd.error_count_pc_skip)) + " Servo:"+str(
                                "{:}".format(mrd.error_count_servo_skip))+" PCframe:"+str(mrd.loop_count)+" BOARDframe:"+str(mrd.frame_sync_r_recv)+" "+str(int(mrd.loop_count/now))+"Hz"

                        # 今回受信のシーケンス番号を次回比較用にキープ
                        mrd.frame_sync_r_last = mrd.frame_sync_r_recv

                        # Selfモード: 累積絶対タイマーで100Hz維持(ドリフト自己補正)
                        if mrd.flag_self_mode:
                            _remain = _self_next_frame_time - time.time()
                            if _remain > 0.0:
                                _select.select([], [], [], _remain)
                            _self_next_frame_time += 0.01

# ------------------------------------------------------------------------
# [ 8 ] : シーケンス番号が更新されていなければ待機して[1-1]]に戻る
# ------------------------------------------------------------------------
                else:
                    time.sleep(0.001)


# ================================================================================================================
# ---- 関 数 各 種 ---------------------------------------------------------------------------------------------------
# ================================================================================================================

# ctrl+cで終了したときにも確実にソケットを閉じる試み(いまのところ機能していないかも)
def cleanup():
    print("Meridan_console quited.")


# フラグ切り替え用の真偽反転
def flip_number(appdata, string1, string2):
    if appdata == True:
        print(string1)
        return True
    else:
        print(string2)
        return False


# 押しボタンでフラグを立てる
def push_button_flag(string):
    print(string)
    return 1


# [Axis Monitor] ウィンドウのスライダー処理
def set_servo_angle(channel, app_data):
    if channel[3] == "L":
        mrd.s_meridim[int(channel[4:6])*2+21] = int(app_data * 100)
        mrd.s_meridim_motion_f[int(channel[4:6])*2+21] = app_data

        # Trim Settingウィンドウが開かれている場合は, 対応するスライダーを更新
        if mrd.flag_trim_window_open and dpg.does_item_exist(f"Trim_L{channel[4:6]}"):
            dpg.set_value(f"Trim_L{channel[4:6]}", app_data)

    if channel[3] == "R":
        mrd.s_meridim[int(channel[4:6])*2+51] = int(app_data * 100)
        mrd.s_meridim_motion_f[int(channel[4:6])*2+51] = app_data

        # Trim Settingウィンドウが開かれている場合は, 対応するスライダーを更新
        if mrd.flag_trim_window_open and dpg.does_item_exist(f"Trim_R{channel[4:6]}"):
            dpg.set_value(f"Trim_R{channel[4:6]}", app_data)


# [Axis Monitor] ウィンドウのTarget, Actual 切り替えラジオボタン処理
def change_display_mode(sender, app_data, user_data):
    chosen_option = dpg.get_value(sender)
    if chosen_option == "Target":
        mrd.flag_display_mode = 1
        mrd.flag_ros1_output_mode = 1
    elif chosen_option == "Actual":
        mrd.flag_display_mode = 0
        mrd.flag_ros1_output_mode = 0


def send_raw_zero_temp():
    """Trim Settingウィンドウ用のRaw Zero。トリムスライダー値を保持したままangle=0を送信する。
    Trim Zeroボタンで編集中のトリム位置に戻れる。"""
    for i in range(MRD_SERVO_SLOTS):
        l_ix = MRD_L_ORIG_IDX + 1 + i * 2
        r_ix = MRD_R_ORIG_IDX + 1 + i * 2
        mrd.s_meridim[l_ix] = 0
        mrd.s_meridim[r_ix] = 0
        mrd.s_meridim_motion_f[l_ix] = 0
        mrd.s_meridim_motion_f[r_ix] = 0
        mrd.s_meridim_motion_keep_f[l_ix] = 0
        mrd.s_meridim_motion_keep_f[r_ix] = 0
        dpg.set_value(f"ID L{i}", 0)
        dpg.set_value(f"ID R{i}", 0)
    print("Raw Zero: Servos moved to mechanical zero (trim sliders preserved).")


def send_trim_zero_restore():
    """現在のトリムスライダー値からサーボ位置を復元する(Raw Zeroから編集状態に戻る)。"""
    for i in range(MRD_SERVO_SLOTS):
        for side, orig_idx in [("L", MRD_L_ORIG_IDX), ("R", MRD_R_ORIG_IDX)]:
            servo_ix = f"{side}{i}"
            tag = f"Trim_{servo_ix}"
            trim_val = dpg.get_value(tag) if dpg.does_item_exist(tag) else 0.0
            cw = -1 if mrd.servo_direction.get(servo_ix, False) else 1
            pos = trim_val * cw
            mrd.s_meridim[orig_idx + 1 + i * 2] = int(pos * 100)
            mrd.s_meridim_motion_f[orig_idx + 1 + i * 2] = pos
            mrd.s_meridim_motion_keep_f[orig_idx + 1 + i * 2] = pos
            dpg.set_value(f"ID {servo_ix}", trim_val)
    print("Trim Zero: Restored servo positions from trim sliders.")


# [Message] ウィンドウの送信データ表示処理
def set_disp_send():
    if mrd.flag_disp_send == 0:
        mrd.flag_disp_send = 2
        print("Display send meridim data.")
    else:
        mrd.flag_disp_send = 0
        print("Stop to display send meridim data.")


# [Message] ウィンドウの受信データ表示処理
def set_disp_rcvd():
    if mrd.flag_disp_rcvd == 0:
        mrd.flag_disp_rcvd = 2
        print("Display received meridim data.")
    else:
        mrd.flag_disp_rcvd = 0
        print("Stop to display received meridim data.")


# [Message] ウィンドウの reset cycle ボタン処理
def reset_cycle():  # サイクルのリセット
    mrd.frag_reset_cycle = True


# [Message] ウィンドウの reset counter ボタン処理
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
    mrd.frag_reset_errors = True
    mrd.error_servo_id = "None"
    mrd.start = time.time()


# [Button Input] ウィンドウ のリモコンボタン処理
def pad_btn_panel_on(sender, app_data, user_data):
    if (mrd.pad_button_panel_short[0] & user_data) == 0:
        mrd.pad_button_panel_short[0] = mrd.pad_button_panel_short[0] | user_data
    else:
        mrd.pad_button_panel_short[0] = mrd.pad_button_panel_short[0] ^ user_data


# [sensor monitor] ウィンドウのSetYawボタン処理
def set_yaw_center():  # IMUのヨー軸センターリセットフラグをcommand_send_trial回上げる(コマンドをcommand_send_trial回送信する)
    mrd.flag_update_yaw = mrd.command_send_trial


# [command] ウィンドウのPowerフラグ処理(サーボのオンオフ)
def set_servo_power(sender, app_data, user_data):
    if app_data:
        mrd.flag_servo_power = 2
        print("Servo Power ON")
    else:
        mrd.flag_servo_power = -1
        print("Servo Power OFF")

    # Trim Settingウィンドウが開かれている場合は, そちらのPowerチェックボックスも更新
    if mrd.flag_trim_window_open and dpg.does_item_exist("Power_Trim"):
        dpg.set_value("Power_Trim", app_data)


# [command] ウィンドウのDemoフラグ処理
def set_demo_action(sender, app_data, user_data):  # チェックボックスに従いアクション送信フラグをオンオフ
    mrd.flag_demo_action = flip_number(
        app_data, "Start DEMO motion data streaming.", "Quit DEMO motion data streaming.")


# [command] ウィンドウのPythonフラグ処理
def set_python_action(sender, app_data, user_data):
    mrd.flag_python_action = flip_number(
        app_data, "Start python motion data streaming.", "Quit python motion data streaming.")


# [command] ウィンドウのEnableフラグ処理
def set_enable(sender, app_data, user_data):
    mrd.flag_enable_send_made_data = flip_number(
        app_data, "Start sending data to ESP32.", "Quit sending data to ESP32.")

    # Trim Settingウィンドウが開かれている場合は, そちらのEnableチェックボックスも更新
    if mrd.flag_trim_window_open and dpg.does_item_exist("Enable_Trim"):
        dpg.set_value("Enable_Trim", app_data)


# [command] SelfモードのON/OFF
def set_self_mode(sender, app_data, user_data):
    mrd.flag_self_mode = app_data
    if not app_data:
        # Restore flow/step state from radio button when disabling self mode
        try:
            if dpg.get_value("transaction_mode") == "Step":
                mrd.flag_set_flow_or_step = -1
                mrd.flag_stop_flow = True
            else:
                mrd.flag_set_flow_or_step = 1
                mrd.flag_stop_flow = False
        except Exception:
            mrd.flag_set_flow_or_step = 1
            mrd.flag_stop_flow = False


# [command] ウィンドウのROS1データ送信モードをtarget/actualに切り替え
def change_ros1_output_mode(sender, app_data, user_data):
    mrd.flag_ros1_output_mode = flip_number(
        app_data, "Set target data(send data) as ROS1 publish.", "Set actual data(received data) as ROS1 publish.")


# [command] ウィンドウのROS1パブリッシュをオンオフ
def ros1_pub():
    if mrd.flag_ros1_pub == 0:
        mrd.flag_ros1_pub = 1
        print("Start publishing ROS1 joint_states.")
    else:
        mrd.flag_ros1_pub = 0
        print("Quit publishing ROS1 joint_states.")


# [command] ウィンドウのROS1サブスクライブをオンオフ
def ros1_sub():
    if mrd.flag_ros1_sub == 0:
        mrd.flag_ros1_sub = 1
        print("Start subscribing ROS1 joint_states.")
    else:
        mrd.flag_ros1_sub = 0
        print("Quit publishing ROS1 joint_states.")


# [command] ウィンドウのROS1データ変換
def joinstate_to_meridim(JointState):
    for i in range(11):
        mrd.s_meridim_js_sub_f[MRD_L_ORIG_IDX + 1 + i * 2] = round(
            math.degrees(JointState.position[i])*100)*mrd.jspn[i]
        mrd.s_meridim_js_sub_f[MRD_R_ORIG_IDX + 1 + i * 2] = round(
            math.degrees(JointState.position[11 + i])*100)*mrd.jspn[15 + i]


# [Mini Terminal] ウィンドウのsetボタン処理
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
                    mrd.s_miniterminal_keep[i][0] = int(
                        dpg.get_value(_value_tag_index))
                    mrd.s_miniterminal_keep[i][1] = int(
                        dpg.get_value(_value_tag_data))
                else:
                    # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
                    mrd.s_miniterminal_keep[i][0] = -1
                    print_string = print_string + \
                        "["+str(dpg.get_value(_value_tag_index)) + \
                        "] out of range, "
    print("Set mini tarminal data : ")
    print(print_string[:-2])  # 末尾のカンマ以外を表示


# [Mini Terminal] ウィンドウのSendボタン処理
def set_terminal_continuous_on(sender, app_data):  # ボタン押下でset_flowフラグをオン
    if app_data:
        mrd.flag_terminal_mode_send = 2
        print("Start to send miniterminal data.")
    else:
        mrd.flag_terminal_mode_send = 0
        print("Stop to send miniterminal data.")
        for i in range(8):
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_miniterminal_keep[i][0] = -1


# [Mini Terminal] ウィンドウのSendボタン処理
def set_terminal_send_on():  # ボタン押下でset_flowフラグをオン
    if mrd.flag_terminal_mode_send == 0:
        mrd.flag_terminal_mode_send = 2
        print("Set to send miniterminal data.")
    else:
        mrd.flag_terminal_mode_send = 0
        # print("Stop to send miniterminal data.")
        for i in range(8):
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_miniterminal_keep[i][0] = -1
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_miniterminal_keep[i][1] = 0


# [Mini Terminal] ウィンドウのset&sendボタン処理
def set_and_send_miniterminal_data():  # ミニターミナルのセットボタンが押下されたら送信データをセットする
    set_miniterminal_data()
    set_terminal_send_on()
    mrd.flag_send_miniterminal_data_once = 1    # ミニターミナルの値を1回送信する


# [Mini Terminal] ウィンドウのFlow, Step 切り替えラジオボタン処理
def set_transaction_mode(sender, app_data):
    if app_data == "Flow":  # ボタン押下でset_flowフラグをオン
        mrd.flag_set_flow_or_step = 2
        # mrd.flag_flow_switch = True
        mrd.flag_stop_flow = False
    elif app_data == "Step":  # ボタン押下でset_stepフラグをオン
        mrd.flag_set_flow_or_step = -2
        # mrd.flag_flow_switch = True
        mrd.flag_stop_flow = True


# [Mini Terminal] ウィンドウの Next frame ボタン処理
def send_data_step_frame():
    if mrd.flag_self_mode and mrd.flag_set_flow_or_step >= 0:
        # Self mode + Flow mode: activate step behavior so [3] will block next frame
        mrd.flag_set_flow_or_step = -1
    mrd.flag_stop_flow = False  # release current frame


def redis_pub():
    """RedisへのパブリッシュをON/OFF"""
    if mrd.flag_redis_pub:
        mrd.flag_redis_pub = False
    else:
        mrd.flag_redis_pub = True


def redis_sub():
    """RedisのサブスクライブをON/OFF"""
    if mrd.flag_redis_sub:
        mrd.flag_redis_sub = False
    else:
        mrd.flag_redis_sub = True


# ---- Valkey Connect ウィンドウ用 -----------------------------------------------------------------------

_BOARD_IP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "board_ip.txt")
_VALKEY_DEFAULT_KEYS = ["merikey_real_pub", "meridis_real_pub"]
_VALKEY_SERVER = "/opt/homebrew/bin/valkey-server"
_VALKEY_CLI    = "/opt/homebrew/bin/valkey-cli"


def _valkey_load_keys():
    """board_ip.txt から VALKEY_PUBLISH_KEYS を読み込む。なければデフォルトを返す。"""
    try:
        with open(_BOARD_IP_FILE, 'r') as f:
            for line in f:
                if line.startswith('VALKEY_PUBLISH_KEYS='):
                    val = line.split('=', 1)[1].strip().strip('"')
                    keys = [k.strip() for k in val.split(',') if k.strip()]
                    if keys:
                        return keys
    except Exception:
        pass
    return list(_VALKEY_DEFAULT_KEYS)


def _valkey_save_keys(keys):
    """Valkeyキーリストを board_ip.txt に保存。"""
    line_new = f'VALKEY_PUBLISH_KEYS="{",".join(keys)}"\n'
    try:
        lines = open(_BOARD_IP_FILE).readlines()
        found = False
        for i, l in enumerate(lines):
            if l.startswith('VALKEY_PUBLISH_KEYS='):
                lines[i] = line_new
                found = True
                break
        if not found:
            lines.append(line_new)
        with open(_BOARD_IP_FILE, 'w') as f:
            f.writelines(lines)
    except Exception as e:
        print(f"[Valkey] Config save error: {e}")


def valkey_start():
    """Valkeyサーバーをバックグラウンドで起動し、Publishキーを hset 初期化。"""
    def _run():
        try:
            # 既に起動中であればサーバー起動をスキップ
            try:
                _c = redis.Valkey(host=REDIS_HOST, port=REDIS_PORT,
                                  socket_connect_timeout=0.5)
                _c.ping()
                _c.close()
                print("[Valkey] Server already running. Skipping start.")
                _init_keys()  # _reconnect_transfer() も内部で呼ばれる
                _valkey_update_status()
                return
            except Exception:
                pass

            cmd = f'{_VALKEY_SERVER} --save "" --dir /tmp --dbfilename valkey_meridis.rdb'
            proc = subprocess.Popen(cmd, shell=True,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                    start_new_session=True)
            print(f"[Valkey] Server starting (PID {proc.pid})...")
            # 起動完了を最大5秒待つ
            for _ in range(50):
                time.sleep(0.1)
                if proc.poll() is not None:
                    print(f"[Valkey] Server exited early (rc={proc.returncode}).")
                    return
                try:
                    client = redis.Valkey(host=REDIS_HOST, port=REDIS_PORT,
                                         socket_connect_timeout=0.3)
                    client.ping()
                    client.close()
                    break
                except Exception:
                    continue
            else:
                print("[Valkey] Server did not respond within 5s.")
                return
            _init_keys()
            _valkey_update_status()
        except Exception as e:
            print(f"[Valkey] Start error: {e}")
            _valkey_update_status()

    def _reconnect_transfer():
        """redis_transfer が未接続なら再接続を試みる。"""
        if not redis_transfer.is_connected:
            try:
                redis_transfer.redis_client.ping()
                redis_transfer.is_connected = True
            except Exception:
                pass

    def _init_keys():
        keys = list(dpg.get_item_configuration("connect_publish_combo")["items"])
        client = redis.Valkey(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)
        for key in keys:
            if not client.exists(key):
                client.hset(key, mapping={str(i): "0" for i in range(90)})
            print(f"[Valkey] Key ready: '{key}'")
        client.close()
        _reconnect_transfer()

    threading.Thread(target=_run, daemon=True).start()


def valkey_reset():
    """Valkey の全データをフラッシュ。"""
    try:
        client = redis.Valkey(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True,
                              socket_connect_timeout=1.0)
        client.flushall()
        client.close()
        print("[Valkey] FLUSHALL executed.")
    except Exception as e:
        print(f"[Valkey] Reset error: {e}")


def valkey_shutdown():
    """Valkey サーバーをシャットダウン。"""
    for cmd in [[_VALKEY_CLI, 'shutdown'], ['redis-cli', 'shutdown']]:
        try:
            subprocess.run(cmd, timeout=3,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[Valkey] Shutdown via {cmd[0]}.")
            threading.Thread(target=lambda: (time.sleep(0.5), _valkey_update_status()),
                             daemon=True).start()
            return
        except Exception:
            continue
    print("[Valkey] Shutdown failed.")
    _valkey_update_status()


def valkey_publish_changed(sender, app_data):
    """Publishドロップダウン変更: redis_transfer の書き込み先キーを切り替え。"""
    redis_transfer.redis_key = app_data
    print(f"[Valkey] Publish key → '{app_data}'")


def valkey_subscribe_changed(sender, app_data):
    """Subscribeドロップダウン変更: redis_receiver の受信先キーを切り替え。"""
    redis_receiver.redis_key = app_data
    print(f"[Valkey] Subscribe key → '{app_data}'")


def valkey_key_add():
    """テキスト入力のキーを Publish/Subscribe ドロップダウンに追加し保存。"""
    new_key = dpg.get_value("connect_key_input").strip()
    if not new_key:
        return
    items = list(dpg.get_item_configuration("connect_publish_combo")["items"])
    if new_key not in items:
        items.append(new_key)
        dpg.configure_item("connect_publish_combo", items=items)
        dpg.configure_item("connect_subscribe_combo", items=items)
        _valkey_save_keys(items)
        print(f"[Valkey] Key added: '{new_key}'")


def valkey_key_del():
    """選択中の Publish キーを両ドロップダウンから削除し保存。"""
    selected = dpg.get_value("connect_publish_combo")
    items = list(dpg.get_item_configuration("connect_publish_combo")["items"])
    if selected in items:
        items.remove(selected)
        dpg.configure_item("connect_publish_combo", items=items)
        dpg.configure_item("connect_subscribe_combo", items=items)
        new_sel = items[0] if items else ""
        dpg.set_value("connect_publish_combo", new_sel)
        redis_transfer.redis_key = new_sel
        _valkey_save_keys(items)
        print(f"[Valkey] Key removed: '{selected}'")


def valkey_keys_refresh():
    """Valkeyの KEYS * を取得してモーダルのテキストを更新。"""
    try:
        client = redis.Valkey(host=REDIS_HOST, port=REDIS_PORT,
                              socket_connect_timeout=0.5, decode_responses=True)
        keys = sorted(client.keys('*'))
        client.close()
        text = '\n'.join(keys) if keys else "(no keys)"
    except Exception as e:
        text = f"Not connected:\n{e}"
    dpg.set_value("valkey_keys_text", text)


def valkey_disp_keys():
    """Disp Keys モーダルを開き、キー一覧を表示。"""
    valkey_keys_refresh()
    dpg.configure_item("valkey_keys_modal", show=True)


def _valkey_close_disp_keys():
    """Disp Keys モーダルを閉じ、取得済みキーを Subscribe ドロップダウンに反映。"""
    text = dpg.get_value("valkey_keys_text")
    keys = [k.strip() for k in text.splitlines()
            if k.strip() and not k.startswith("(") and not k.startswith("Not")]
    if keys:
        current = dpg.get_value("connect_subscribe_combo")
        dpg.configure_item("connect_subscribe_combo", items=keys)
        if current in keys:
            dpg.set_value("connect_subscribe_combo", current)
        else:
            dpg.set_value("connect_subscribe_combo", keys[0])
            redis_receiver.redis_key = keys[0]
    dpg.configure_item("valkey_keys_modal", show=False)


# ---------- Valkey server status indicator ----------

def _valkey_check_online():
    """Port 6379 へのTCP接続でValkeyの起動状態を確認(軽量)。"""
    try:
        conn = socket.create_connection(("127.0.0.1", REDIS_PORT), timeout=0.2)
        conn.close()
        return True
    except Exception:
        return False


def _valkey_update_status():
    """valkey_status_text の表示色・テキストを最新状態に更新。"""
    online = _valkey_check_online()
    try:
        if online:
            dpg.set_value("valkey_status_text", "Valkey Server On")
            dpg.configure_item("valkey_status_text", color=(64, 200, 224, 255))
        else:
            dpg.set_value("valkey_status_text", "Valkey Server Off")
            dpg.configure_item("valkey_status_text", color=(128, 128, 128, 255))
    except Exception:
        pass


def _valkey_status_loop():
    """1秒ごとにValkeyの起動状態を確認してUIを更新するバックグラウンドスレッド。"""
    while mrd.running:
        _valkey_update_status()
        time.sleep(1.0)


# ================================================================================================================
# ---- ゲームパッド処理 --------------------------------------------------------------------------------------------
# ================================================================================================================

# SDL2/pygame button index → Meridim uint16 bitmask (PS4 on macOS)
_PYGAME_BTN_TO_MERIDIM = {
    0:  16384,  # Cross(X)     → R_DOWN
    1:  8192,   # Circle(O)    → R_RIGHT
    2:  32768,  # Square(Sq)   → R_LEFT
    3:  4096,   # Triangle(Tri)→ R_UP
    4:  1,      # Share        → SELECT
    6:  8,      # Options      → START
    7:  2,      # L3           → bit1
    8:  4,      # R3           → bit2
    9:  1024,   # L1           → L1
    10: 2048,   # R1           → R1
    11: 16,     # Up  (D-pad as button)
    12: 64,     # Down
    13: 128,    # Left
    14: 32,     # Right
}

_gamepad_joysticks: dict = {}  # {index: pygame.joystick.Joystick}


def pad_player_changed(sender, app_data, user_data):
    """プレイヤー選択プルダウンのコールバック。"""
    items = ["None", "Player 1", "Player 2", "Player 3", "Player 4", "Player 5", "Player 6"]
    try:
        idx = items.index(app_data) - 1  # "None"=-1, "Player 1"=0, ...
    except ValueError:
        idx = -1
    mrd.pad_gamepad_player = idx
    if idx < 0:
        mrd.pad_gamepad_buttons[0] = 0
        mrd.pad_gamepad_axes[:] = 0.0
        mrd.pad_gamepad_connected = False


def _gamepad_poll_loop():
    """ゲームパッドの状態を100Hzでポーリングするバックグラウンドスレッド。"""
    global _gamepad_joysticks
    while mrd.running:
        if not _pygame_available:
            time.sleep(0.5)
            continue
        try:
            _pygame.event.pump()
            count = _pygame.joystick.get_count()

            # 接続/切断に合わせてJoystickオブジェクトを管理
            for i in range(count):
                if i not in _gamepad_joysticks:
                    joy = _pygame.joystick.Joystick(i)
                    joy.init()
                    _gamepad_joysticks[i] = joy
            for i in list(_gamepad_joysticks.keys()):
                if i >= count:
                    try:
                        _gamepad_joysticks[i].quit()
                    except Exception:
                        pass
                    del _gamepad_joysticks[i]

            player = mrd.pad_gamepad_player
            if player < 0 or player not in _gamepad_joysticks:
                mrd.pad_gamepad_buttons[0] = 0
                mrd.pad_gamepad_axes[:] = 0.0
                mrd.pad_gamepad_connected = False
                time.sleep(0.02)
                continue

            joy = _gamepad_joysticks[player]

            # ボタン → Meridimビットマスク変換
            bitmask = np.uint16(0)
            n_buttons = joy.get_numbuttons()
            for btn_idx, meridim_bit in _PYGAME_BTN_TO_MERIDIM.items():
                if btn_idx < n_buttons and joy.get_button(btn_idx):
                    bitmask = np.uint16(bitmask | np.uint16(meridim_bit))

            # ハット(D-pad) → ビットマスク
            if joy.get_numhats() > 0:
                hx, hy = joy.get_hat(0)
                if hx == -1: bitmask = np.uint16(bitmask | np.uint16(128))  # L_LEFT
                if hx ==  1: bitmask = np.uint16(bitmask | np.uint16(32))   # L_RIGHT
                if hy ==  1: bitmask = np.uint16(bitmask | np.uint16(16))   # L_UP
                if hy == -1: bitmask = np.uint16(bitmask | np.uint16(64))   # L_DOWN

            # アナログ軸
            n_axes = joy.get_numaxes()
            axes = np.zeros(6, dtype=np.float64)
            if n_axes > 0: axes[0] = joy.get_axis(0)   # LX
            if n_axes > 1: axes[1] = joy.get_axis(1)   # LY
            if n_axes > 2: axes[2] = joy.get_axis(2)   # RX
            if n_axes > 3: axes[3] = joy.get_axis(3)   # RY
            if n_axes > 4:
                l2 = joy.get_axis(4)  # -1.0(off) to 1.0(full)
                axes[4] = l2
                if l2 > 0.0:
                    bitmask = np.uint16(bitmask | np.uint16(256))  # L2 digital
            if n_axes > 5:
                r2 = joy.get_axis(5)
                axes[5] = r2
                if r2 > 0.0:
                    bitmask = np.uint16(bitmask | np.uint16(512))  # R2 digital

            mrd.pad_gamepad_buttons[0] = bitmask
            mrd.pad_gamepad_axes[:] = axes
            mrd.pad_gamepad_connected = True

        except Exception:
            mrd.pad_gamepad_buttons[0] = 0
            mrd.pad_gamepad_axes[:] = 0.0
            mrd.pad_gamepad_connected = False

        time.sleep(0.01)  # 100Hz


# ================================================================================================================
# ---- dearpyguiによるコンソール画面描写 -----------------------------------------------------------------------------
# ================================================================================================================
def main():
    while mrd.running:

        # dpg描画処理1 ==========================================================
        dpg.create_context()
        # dpg.create_viewport(title=TITLE_VERSION, width=853, height=540) #mac/ubuntu
        dpg.create_viewport(title=TITLE_VERSION, width=870, height=580)  # win


# ------------------------------------------------------------------------
# [ Axis Monitor ] : サーボ位置モニタリング用のウィンドウ(表示位置:上段/左側)
# ------------------------------------------------------------------------
        # [ Axis Monitor ] : サーボ位置モニタリング用のウィンドウ(表示位置:上段/左側)
        with dpg.window(label="Axis Monitor", width=250, height=370, pos=[5, 5]):
            with dpg.group(label='RightSide'):
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID R"+str(i), label="R"+str(i),
                                         max_value=180, min_value=-180, callback=set_servo_angle, pos=[10, 35 + i * 20], width=80)

            with dpg.group(label='LeftSide'):
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID L"+str(i), label="L"+str(i),
                                         max_value=180, min_value=-180, callback=set_servo_angle, pos=[135, 35 + i * 20], width=80)

            dpg.add_button(label="Zero", callback=load_trimdata_from_eeprom_to_board, pos=[10, 340], width=40)
            dpg.add_button(label="Trim", callback=open_trim_window, pos=[55, 340], width=40)
            dpg.add_radio_button(label="display_mode", items=["Target", "Actual"], callback=change_display_mode,
                                 default_value="Actual", pos=[100, 340], horizontal=True)


# ------------------------------------------------------------------------
# [ Message ] : メッセージ表示用ウィンドウ(表示位置:下段/左側)
# ------------------------------------------------------------------------
        with dpg.window(label="Messege", width=590, height=155, pos=[5, 380]):

            dpg.add_text("disp_send ", pos=[383, 53])
            dpg.add_checkbox(tag="disp_send", callback=set_disp_send, pos=[452, 53])
            dpg.add_text("disp_rcvd ", pos=[483, 53])
            dpg.add_checkbox(tag="disp_rcvd", callback=set_disp_rcvd, pos=[551, 53])
            dpg.add_button(label="ResetCycle", callback=reset_cycle, width=80, pos=[390, 28])
            dpg.add_button(label="ResetCounter", callback=reset_counter, width=90, pos=[480, 28])
            dpg.add_text(mrd.message0, tag="DispMessage0")
            dpg.add_text(mrd.message1, tag="DispMessage1")
            dpg.add_text(mrd.message2, tag="DispMessage2")
            dpg.add_text(mrd.message3, tag="DispMessage3")
            dpg.add_text(mrd.message4, tag="DispMessage4")

# ------------------------------------------------------------------------
# [ Sensor Monitor ] : センサー値モニタリング用ウィンドウ(表示位置:上段/中央)
# ------------------------------------------------------------------------
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
                dpg.add_button(label="SetYaw", callback=set_yaw_center, width=50, pos=[270, 148])

# ------------------------------------------------------------------------
# [ Command ] : コマンド送信/リモコン値表示用ウィンドウ(表示位置:中段/中央)
# ------------------------------------------------------------------------
        with dpg.window(label="Command", width=335, height=190, pos=[260, 185]):
            dpg.add_checkbox(label="Power", tag="Power", callback=set_servo_power, pos=[100, 27])
            dpg.add_checkbox(label="Enable", tag="Enable", callback=set_enable, pos=[100, 53])
            dpg.add_checkbox(label="Demo", tag="Action", callback=set_demo_action, pos=[116, 76])
            dpg.add_checkbox(label="Python", tag="python", callback=set_python_action, pos=[116, 99])

            dpg.add_text("NoESP", pos=[20, 27])
            dpg.add_checkbox(tag="Self", callback=set_self_mode, pos=[57, 27])

            dpg.add_text("ESP32 ->", pos=[20, 60])
            dpg.add_text("ESP32 <-", pos=[20, 83])

            dpg.add_checkbox(tag="ROS1pub", callback=ros1_pub, pos=[283, 27])
            dpg.add_text("-> ROS1", pos=[210, 27])
            dpg.add_checkbox(tag="ROS1sub", callback=ros1_sub, pos=[283, 51])
            dpg.add_text("<- ROS1", pos=[210, 51])
            dpg.add_checkbox(tag="RedisPub", callback=redis_pub, pos=[283, 76])
            dpg.add_text("-> Valkey ", pos=[210, 76])
            dpg.add_checkbox(tag="Redis", callback=redis_sub, pos=[283, 99])
            dpg.add_text("<- Valkey ", pos=[210, 99])


            dpg.draw_rectangle(pmin=[80, -4], pmax=[190, 95], color=(
                100, 100, 100, 255), thickness=1.0, fill=(0, 0, 0, 0))
            dpg.draw_line(p1=[84, 22], p2=[186, 22], color=(
                100, 100, 100, 255), thickness=1.0)

            dpg.add_text("Control Pad Monitor", pos=[10, 123])
            dpg.add_text("button", tag="pad_button", pos=[170, 123])
            dpg.add_slider_int(default_value=0, tag="pad_Lx", label="Lx",
                               max_value=127, min_value=-127, pos=[10, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Ly", label="Ly",
                               max_value=127, min_value=-127, pos=[90, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Rx", label="Rx",
                               max_value=127, min_value=-127, pos=[170, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Ry", label="Ry",
                               max_value=127, min_value=-127, pos=[250, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_L2v", label="L2v",
                               max_value=255, min_value=0, pos=[90, 163], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_R2v", label="R2v",
                               max_value=255, min_value=0, pos=[170, 163], width=40)

# ------------------------------------------------------------------------
# [ Button Input ] : リモコン入力コンパネ用ウィンドウ(表示位置:上段/右側)
# ------------------------------------------------------------------------
        with dpg.window(label="Button Input", width=248, height=155, pos=[600, 5]):
            dpg.add_text("Pad Not connected", tag="pad_status_text",
                         color=(128, 128, 128, 255), pos=[62, 20])
            dpg.add_combo(tag="pad_player_combo",
                          items=["None", "Player 1", "Player 2", "Player 3",
                                 "Player 4", "Player 5", "Player 6"],
                          default_value="None",
                          width=130, pos=[59, 43], callback=pad_player_changed)
            dpg.add_checkbox(tag="Btn_L2",      callback=pad_btn_panel_on, user_data=256, pos=[15, 38])
            dpg.add_checkbox(tag="Btn_L1",      callback=pad_btn_panel_on, user_data=1024, pos=[15, 60])
            dpg.add_checkbox(tag="Btn_L_UP",    callback=pad_btn_panel_on, user_data=16, pos=[42, 80])
            dpg.add_checkbox(tag="Btn_L_DOWN",  callback=pad_btn_panel_on, user_data=64, pos=[42, 124])
            dpg.add_checkbox(tag="Btn_L_LEFT",  callback=pad_btn_panel_on, user_data=128, pos=[20, 102])
            dpg.add_checkbox(tag="Btn_L_RIGHT", callback=pad_btn_panel_on, user_data=32, pos=[64, 102])
            dpg.add_checkbox(tag="Btn_SELECT",  callback=pad_btn_panel_on, user_data=1, pos=[100, 102])
            dpg.add_checkbox(tag="Btn_START",   callback=pad_btn_panel_on, user_data=8, pos=[130, 102])
            dpg.add_checkbox(tag="Btn_R2",      callback=pad_btn_panel_on, user_data=512, pos=[215, 38])
            dpg.add_checkbox(tag="Btn_R1",      callback=pad_btn_panel_on, user_data=2048, pos=[215, 60])
            dpg.add_checkbox(tag="Btn_R_UP",    callback=pad_btn_panel_on, user_data=4096, pos=[188, 80])
            dpg.add_checkbox(tag="Btn_R_DOWN",  callback=pad_btn_panel_on, user_data=16384, pos=[188, 124])
            dpg.add_checkbox(tag="Btn_R_LEFT",  callback=pad_btn_panel_on, user_data=32768, pos=[166, 102])
            dpg.add_checkbox(tag="Btn_R_RIGHT", callback=pad_btn_panel_on, user_data=8192, pos=[210, 102])

# ------------------------------------------------------------------------
# [ Mini Terminal ] : コマンド送信用ミニターミナル(表示位置:中段/右側)
# ------------------------------------------------------------------------
        with dpg.window(label="Mini Terminal", width=248, height=203, pos=[600, 165]):
            # with dpg.group(label='LeftSide'):
            dpg.add_text("Index", pos=[15, 25])
            dpg.add_text("Data", pos=[60, 25])
            dpg.add_input_text(tag="s_index0", decimal=True, default_value="0", width=40, pos=[15, 45])
            dpg.add_input_text(tag="s_data0",  decimal=True, default_value=str(MSG_SIZE), width=60, pos=[60, 45])
            dpg.add_input_text(tag="s_index1", decimal=True, default_value="", width=40, pos=[15, 70])
            dpg.add_input_text(tag="s_data1",  decimal=True, default_value="", width=60, pos=[60, 70])
            dpg.add_input_text(tag="s_index2", decimal=True, default_value="", width=40, pos=[15, 95])
            dpg.add_input_text(tag="s_data2",  decimal=True, default_value="", width=60, pos=[60, 95])
            dpg.add_input_text(tag="s_index3", decimal=True, default_value="", width=40, pos=[15, 120])
            dpg.add_input_text(tag="s_data3",  decimal=True, default_value="", width=60, pos=[60, 120])
            dpg.add_text("Index", pos=[130, 25])
            dpg.add_text("Data", pos=[175, 25])
            dpg.add_input_text(tag="s_index4", decimal=True, default_value="", width=40, pos=[130, 45])
            dpg.add_input_text(tag="s_data4",  decimal=True, default_value="", width=60, pos=[175, 45])
            dpg.add_input_text(tag="s_index5", decimal=True, default_value="", width=40, pos=[130, 70])
            dpg.add_input_text(tag="s_data5",  decimal=True, default_value="", width=60, pos=[175, 70])
            dpg.add_input_text(tag="s_index6", decimal=True, default_value="", width=40, pos=[130, 95])
            dpg.add_input_text(tag="s_data6",  decimal=True, default_value="", width=60, pos=[175, 95])
            dpg.add_input_text(tag="s_index7", decimal=True, default_value="", width=40, pos=[130, 120])
            dpg.add_input_text(tag="s_data7",  decimal=True, default_value="", width=60, pos=[175, 120])
            dpg.add_button(label="Set", callback=set_miniterminal_data, pos=[136, 148])
            dpg.add_button(label="Set&Send", callback=set_and_send_miniterminal_data, pos=[171, 148])
            dpg.add_text("Continuous ", pos=[140, 175])
            dpg.add_checkbox(tag="SendContinuously",callback=set_terminal_continuous_on, pos=[215, 175])
            dpg.add_radio_button(["Flow", "Step"], tag="transaction_mode", pos=[10, 148],
                                 callback=set_transaction_mode, default_value="Flow", horizontal=True)
            dpg.add_button(label=" Next frame ", pos=[15, 175], callback=send_data_step_frame)  # 右下に設置

# ------------------------------------------------------------------------
# [ Connect ] : 接続設定ウィンドウ(右下)
# ------------------------------------------------------------------------
        with dpg.window(label="Valkey", width=248, height=162, pos=[600, 373]):
            _vk_keys = _valkey_load_keys()
            dpg.add_button(label="Start Valkey", width=95, pos=[10,  33], callback=valkey_start)
            dpg.add_button(label="Reset",         width=50, pos=[110, 33], callback=valkey_reset)
            dpg.add_button(label="Shutdown",      width=70, pos=[165, 33], callback=valkey_shutdown)
            dpg.add_text("Publish",   pos=[10, 60])
            dpg.add_combo(tag="connect_publish_combo", items=_vk_keys,
                          default_value=_vk_keys[0] if _vk_keys else "",
                          width=153, pos=[82, 57],
                          callback=valkey_publish_changed)
            # 起動時にredis_transferの書き込み先をドロップダウン初期値に同期
            redis_transfer.redis_key = _vk_keys[0] if _vk_keys else REDIS_KEY_WRITE
            dpg.add_input_text(tag="connect_key_input", hint="key name",
                               width=145, pos=[10, 81])
            dpg.add_button(label="Add", width=35, pos=[160, 81], callback=valkey_key_add)
            dpg.add_button(label="Del", width=35, pos=[200, 81], callback=valkey_key_del)
            dpg.add_text("Subscribe", pos=[10, 108])
            _vk_sub_default = _vk_keys[1] if len(_vk_keys) > 1 else (_vk_keys[0] if _vk_keys else "")
            dpg.add_combo(tag="connect_subscribe_combo", items=_vk_keys,
                          default_value=_vk_sub_default,
                          width=153, pos=[82, 105],
                          callback=valkey_subscribe_changed)
            # 起動時にredis_receiverの受信先をドロップダウン初期値に同期
            redis_receiver.redis_key = _vk_sub_default
            dpg.add_text("Valkey Server Off", tag="valkey_status_text",
                         color=(128, 128, 128, 255), pos=[10, 132])
            dpg.add_button(label="Disp Keys", width=90, pos=[145, 129], callback=valkey_disp_keys)

# ------------------------------------------------------------------------
# [ Valkey Keys Modal ] : キー一覧モーダル
# ------------------------------------------------------------------------
        with dpg.window(label="Valkey Keys", tag="valkey_keys_modal",
                        modal=True, show=False, width=280, height=230,
                        pos=[290, 185], no_resize=True):
            with dpg.child_window(width=260, height=155, border=True):
                dpg.add_text(tag="valkey_keys_text", default_value="")
            dpg.add_button(label="Refresh", width=80, callback=valkey_keys_refresh)
            dpg.add_same_line()
            dpg.add_button(label="Close", width=80, callback=_valkey_close_disp_keys)

# dpg描画処理2 =========================================================
        with dpg.value_registry():  # dpg変数値の登録
            dpg.add_int_value(tag="button_data")

        dpg.setup_dearpygui()
        dpg.show_viewport()
        threading.Thread(target=_valkey_status_loop, daemon=True).start()
        threading.Thread(target=_gamepad_poll_loop, daemon=True).start()

# dpg描画内容のデータ更新 ================================================
        while dpg.is_dearpygui_running():
            signal.signal(signal.SIGINT, signal.SIG_DFL)

            # メッセージ欄の表示更新
            dpg.set_value("DispMessage0", mrd.message0)
            dpg.set_value("DispMessage1", mrd.message1)
            dpg.set_value("DispMessage2", mrd.message2)
            dpg.set_value("DispMessage3", mrd.message3)
            dpg.set_value("DispMessage4", mrd.message4)
            # flag_self_mode の変化をチェックボックスに反映
            if dpg.get_value("Self") != mrd.flag_self_mode:
                dpg.set_value("Self", mrd.flag_self_mode)

            # サーボデータとIMUデータの表示更新
            for i in range(0, 15, 1):
                _idld = mrd.d_meridim[MRD_L_ORIG_IDX + 1 + i * 2]
                _idrd = mrd.d_meridim[MRD_R_ORIG_IDX + 1 + i * 2]
                _idsensor = mrd.r_meridim[i+2]/10000
                dpg.set_value("ID L"+str(i), _idld/100)  # サーボIDと数値の表示
                dpg.set_value("ID R"+str(i), _idrd/100)

                if i < 13:  # IMUデータの更新
                    if i < 11:
                        dpg.set_value("mpu"+str(i), _idsensor)
                    else:
                        dpg.set_value("mpu"+str(i), _idsensor*100)

            # ゲームパッド接続状態テキストの更新
            if mrd.pad_gamepad_connected:
                dpg.set_value("pad_status_text", "Pad Connected.")
                dpg.configure_item("pad_status_text", color=(100, 200, 255, 255))
            else:
                dpg.set_value("pad_status_text", "Pad Not connected")
                dpg.configure_item("pad_status_text", color=(128, 128, 128, 255))

            # リモコンデータの表示更新
            pad_button_short = np.array([0], dtype=np.uint16)
            received_button = int(mrd.r_meridim[15]) & 0xFFFF  # int16→uint16変換
            pad_button_short[0] = (received_button
                                   | mrd.pad_button_panel_short[0]
                                   | mrd.pad_gamepad_buttons[0])

            dpg.set_value("pad_button", str(pad_button_short[0]))

            if mrd.pad_gamepad_connected:
                # ゲームパッドの軸値を直接表示
                axes = mrd.pad_gamepad_axes
                dpg.set_value("pad_Lx", int(np.clip(axes[0] * 127, -127, 127)))
                dpg.set_value("pad_Ly", int(np.clip(axes[1] * 127, -127, 127)))
                dpg.set_value("pad_Rx", int(np.clip(axes[2] * 127, -127, 127)))
                dpg.set_value("pad_Ry", int(np.clip(axes[3] * 127, -127, 127)))
                dpg.set_value("pad_L2v", int(np.clip((axes[4] + 1.0) / 2.0 * 255, 0, 255)))
                dpg.set_value("pad_R2v", int(np.clip((axes[5] + 1.0) / 2.0 * 255, 0, 255)))
            else:
                dpg.set_value("pad_Lx", int(mrd.r_meridim_char[33]))
                dpg.set_value("pad_Ly", int(mrd.r_meridim_char[32]))
                dpg.set_value("pad_Rx", int(mrd.r_meridim_char[35]))
                dpg.set_value("pad_Ry", int(mrd.r_meridim_char[34]))
                _padL2val_bin = struct.pack('b', mrd.r_meridim_char[36])
                _padL2val = struct.unpack('B', _padL2val_bin)[0]
                _padR2val_bin = struct.pack('b', mrd.r_meridim_char[37])
                _padR2val = struct.unpack('B', _padR2val_bin)[0]
                dpg.set_value("pad_L2v", int(_padL2val))
                dpg.set_value("pad_R2v", int(_padR2val))

            dpg.set_value("button_data", int(np.int16(pad_button_short[0])))

# ROS1 joint_statesのパブリッシュ =======================================
            global rospy_imported
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
                        ['c_head_yaw',   'l_shoulder_pitch', 'l_shoulder_roll', 'l_elbow_yaw',
                         'l_elbow_pitch', 'l_hipjoint_yaw',   'l_hipjoint_roll', 'l_hipjoint_pitch',
                         'l_knee_pitch', 'l_ankle_pitch',    'l_ankle_roll',
                         'c_chest_yaw',  'r_shoulder_pitch', 'r_shoulder_roll', 'r_elbow_yaw',
                         'r_elbow_pitch', 'r_hipjoint_yaw',   'r_hipjoint_roll', 'r_hipjoint_pitch',
                         'r_knee_pitch', 'r_ankle_pitch',    'r_ankle_roll']
                    js_meridim.position = \
                        [math.radians(mrd.s_meridim_motion_f[21]/100*mrd.jspn[0]),
                         math.radians(mrd.s_meridim_motion_f[23]/100*mrd.jspn[1]),
                         math.radians(mrd.s_meridim_motion_f[25]/100)*mrd.jspn[2],
                         math.radians(mrd.s_meridim_motion_f[27]/100*mrd.jspn[3]),
                         math.radians(mrd.s_meridim_motion_f[29]/100*mrd.jspn[4]),
                         math.radians(mrd.s_meridim_motion_f[31]/100*mrd.jspn[5]),
                         math.radians(mrd.s_meridim_motion_f[33]/100*mrd.jspn[6]),
                         math.radians(mrd.s_meridim_motion_f[35]/100*mrd.jspn[7]),
                         math.radians(mrd.s_meridim_motion_f[37]/100*mrd.jspn[8]),
                         math.radians(mrd.s_meridim_motion_f[39]/100*mrd.jspn[9]),
                         math.radians(mrd.s_meridim_motion_f[41]/100*mrd.jspn[10]),
                         math.radians(mrd.s_meridim_motion_f[51]/100*mrd.jspn[15]),
                         math.radians(mrd.s_meridim_motion_f[53]/100*mrd.jspn[16]),
                         math.radians(mrd.s_meridim_motion_f[55]/100*mrd.jspn[17]),
                         math.radians(mrd.s_meridim_motion_f[57]/100*mrd.jspn[18]),
                         math.radians(mrd.s_meridim_motion_f[59]/100*mrd.jspn[19]),
                         math.radians(mrd.s_meridim_motion_f[61]/100*mrd.jspn[20]),
                         math.radians(mrd.s_meridim_motion_f[63]/100*mrd.jspn[21]),
                         math.radians(mrd.s_meridim_motion_f[65]/100*mrd.jspn[22]),
                         math.radians(mrd.s_meridim_motion_f[67]/100*mrd.jspn[23]),
                         math.radians(mrd.s_meridim_motion_f[69]/100*mrd.jspn[24]),
                         math.radians(mrd.s_meridim_motion_f[71]/100*mrd.jspn[25])]

                    js_meridim.velocity = []
                    # js_meridim.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    js_meridim.effort = []
                    joint_pub.publish(js_meridim)
                    rate.sleep()
                else:
                    print("ROS is not avaliable.")

# ROS1 joint_statesのサブスクライブ =====================================
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

# =====================================================================
            # dpg表示更新処理
            dpg.render_dearpygui_frame()

            time.sleep(0.003)  # CPUの負荷を下げる
        dpg.destroy_context()
        mrd.running = False  # ウィンドウ閉鎖時にmeridian_loopスレッドを停止


# ================================================================================================================
# ---- スレッド処理 ------------------------------------------------------------------------------------------------
# ================================================================================================================
if __name__ == '__main__':  # スレッド2つで送受信と画面描写を並列処理
    thread1 = threading.Thread(target=meridian_loop)  # サブスレッドでフラグ監視・通信処理・計算処理
    thread1.daemon = True
    thread1.start()
    main()  # メインスレッドでdearpygui描写
    thread1.join(timeout=2.0)
