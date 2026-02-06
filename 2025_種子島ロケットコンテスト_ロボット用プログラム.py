#!/usr/bin/env python3
from __future__ import division
import math
from pickle import STOP
from tkinter import NO
from numpy import double
import serial
from serial import Serial
import queue

import time
import subprocess
import RPi.GPIO as GPIO
from datetime import datetime,timedelta
from time import sleep
from multiprocessing import Process, Value, Queue
import os
from smbus import SMBus
import csv


import smbus
from collections import deque



# import pyproj
# grs80 = pyproj.Geod(ellps='GRS80')
# ピンの設定
pins = {
    "PWMAIN1": 32,
    "AIN2": 13,
    "BIN2": 21,
    "PWMBIN1": 33,
}

# モーター制御のための変数
# モーター制御のための変数
PLF = Value('f', 0)#future
PRF = Value('f', 0)#future
PRN = Value('f', 0)#now
PLN = Value('f', 0)#now
MINV = 25
OUTV = 0
#一秒毎に動く距離
PDEV = 90   #1秒間に変わるPWMのパーセント数   
TSTEP = 0.3 #SLEEPとしても扱います
frac=0.7
distance : float= 0.0
dist_Angle:float = 0.0
Move_Angle:float = 0.0
log : str = None
speed : int = None
log_counter = None
#RTK取得用の変数
RTK_SPEED:float = None
RTK_UTC:str = 0
RTK_EAST :double = 0
RTK_NORTH:double = 0
RTK_FIX:int = 0
RTK_ANGLE:float = 0
CMSS : int = 1
NOM_X:float=None
NOM_Y:float=None
end_time = datetime.now()
received_data :str =None

# I2Cバスの識別子
I2C_BUS_ID = 1

# MPL3115A2のI2Cアドレスとレジスタ
MPL3115A2_ADDRESS = 0x60
MPL3115A2_WHOAMI = 0x0C
MPL3115A2_CTRL_REG1 = 0x26
MPL3115A2_CTRL_REG1_SBYB = 0x01
MPL3115A2_CTRL_REG1_OS128 = 0x38
MPL3115A2_CTRL_REG1_BAR = 0x00
MPL3115A2_PT_DATA_CFG = 0x13
MPL3115A2_PT_DATA_CFG_TDEFE = 0x01
MPL3115A2_PT_DATA_CFG_PDEFE = 0x02
MPL3115A2_PT_DATA_CFG_DREM = 0x04
MPL3115A2_REGISTER_PRESSURE_MSB = 0x01

# 磁力計キャリブレーション用の変数
MAX_X = None
MIN_X = None
MAX_Y = None
MIN_Y = None
NOM_X=None
NOM_Y=None
fleg = 1000
# GPIOとPWMの初期設定
GPIO.setmode(GPIO.BOARD)
for pin in pins.values():
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(pins["PWMAIN1"], GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(pins["PWMBIN1"], GPIO.OUT, initial=GPIO.HIGH)
pin1 = GPIO.PWM(pins["PWMAIN1"], fleg)
pin2 = GPIO.PWM(pins["PWMBIN1"], fleg)
pin1.start(0)  # デューティサイクル0で初期化
pin2.start(0)  # デューティサイクル0で初期化

# キューの設定
q = Queue()
q1 = Queue()

# I2Cアドレス
ACCL_ADDR = 0x19  # 加速度計のI2Cアドレス
ACCL_R_ADDR = 0x02  # 加速度計のデータレジスタの開始アドレス
GYRO_ADDR = 0x69  # ジャイロスコープのI2Cアドレス
GYRO_R_ADDR = 0x02  # ジャイロスコープのデータレジスタの開始アドレス
MAG_ADDR = 0x13  # 磁力計のI2Cアドレス
MAG_R_ADDR = 0x42  # 磁力計のデータレジスタの開始アドレス

# I2Cバスを初期化
i2c = SMBus(1)

# シリアルポートのオープン
try:
    TWE = serial.Serial('/dev/ttyS0', 115200, timeout=1)
except serial.SerialException as e:
    print(f"シリアルポートのオープンに失敗しました: {e}")
    exit(1)
   
# subprocess.Popen('/home/tt/RTKLIB/app/str2str/gcc/str2str -in ntrip://guest:guest@160.16.134.72:80/FUKUYAMANT -p 34.502872166 133.348872758 68.6904 -out serial://ttyACM0:115200', shell=True)
RTK_Ser = serial.Serial('/dev/ttyACM0',1)
now = datetime.now().strftime('%Y%m%dT%H%M%S')
fname = '/home/tt/2024/trace_log'+now+'.csv'
files = open(fname,'w')
files.write('RTK_UTC,RTK_EAST,RTK_NORTH,RTK_SPEED,RTK_ANGLE,RTK_FIX,Time,Acc_x,Acc_y,Acc_z,Acc_Magnitude,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z,Heading'+'\n')
log_filename = '/home/tt/2025/trace_log'+now+'.csv'



# I2Cバスの初期化
def init_i2c_bus(bus_id):
    return smbus.SMBus(bus_id)


# デバイス識別子の確認
def check_device(bus, address, whoami_reg):
    whoami = bus.read_i2c_block_data(address, whoami_reg, 1)
    if whoami[0] != 196:
        raise RuntimeError("Adafruit_MPL3115A2が見つかりません")


# デバイスの初期設定
def configure_device(bus, address, ctrl_reg1, config):
    bus.write_byte_data(address, ctrl_reg1, config)


# 気圧データの取得
def read_pressure(bus, address, pressure_msb_reg):
    data = bus.read_i2c_block_data(address, pressure_msb_reg, 3)
    pres = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
    return (pres / 4.0) / 100.0  # 気圧をhPa単位に計算

# サーボモーターの設定
GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)
pwm = GPIO.PWM(16, 50)  # 50Hz
pwm.start(2.5)


# サーボモーターを指定の角度に動かす関数
def move_servo(angle):
    duty_cycle = (angle / 18.0) + 2.5
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"サーボを{angle}度に設定")
    time.sleep(1)


def write2log(f, fprace, press, moji):
    now = datetime.now()
    formatted_timestamp = now.strftime("time%Y-%m-%d %H:%M:%S") + str(now.microsecond // 1000).zfill(3)
    print(formatted_timestamp)
    message = "{},{},{},{}\n".format(formatted_timestamp, fprace, press, moji)
    f.write(message)


# 最後の10回のpresの値を保持するためのdeque（最大長10）
pres_history = deque(maxlen=10)
def dev10(pres):
    # presをpres_historyに追加
    pres_history.append(pres)
    # 最小値と最大値を返す
    return min(pres_history), max(pres_history)


def bmx_setup():
    try:
        # 加速度計の設定
        i2c.write_byte_data(ACCL_ADDR, 0x0F, 0x03)
        i2c.write_byte_data(ACCL_ADDR, 0x10, 0x08)
        i2c.write_byte_data(ACCL_ADDR, 0x11, 0x00)
        time.sleep(0.5)

        # ジャイロスコープの設定
        i2c.write_byte_data(GYRO_ADDR, 0x0F, 0x04)
        i2c.write_byte_data(GYRO_ADDR, 0x10, 0x07)
        i2c.write_byte_data(GYRO_ADDR, 0x11, 0x00)
        time.sleep(0.5)

        # 磁力計の設定
        data = i2c.read_byte_data(MAG_ADDR, 0x4B)
        if data == 0:
            i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
            time.sleep(0.5)
        i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
        i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
        i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
        i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
        i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
        time.sleep(0.5)
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

def read_sensor_data(addr, start_addr, length):
    try:
        return [i2c.read_byte_data(addr, start_addr + i) for i in range(length)]
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return [0] * length

def acc_value():
    data = read_sensor_data(ACCL_ADDR, ACCL_R_ADDR, 6)
    acc_data = [0.0, 0.0, 0.0]
    for i in range(3):
        acc_data[i] = ((data[2 * i + 1] * 256) + (data[2 * i] & 0xF0)) / 16
        if acc_data[i] > 2047:
            acc_data[i] -= 4096
        acc_data[i] *= 0.0098
    return acc_data

def gyro_value():
    data = read_sensor_data(GYRO_ADDR, GYRO_R_ADDR, 6)
    gyro_data = [0.0, 0.0, 0.0]
    for i in range(3):
        gyro_data[i] = (data[2 * i + 1] * 256) + data[2 * i]
        if gyro_data[i] > 32767:
            gyro_data[i] -= 65536
        gyro_data[i] *= 0.0038
    return gyro_data

def mag_value():
    data = read_sensor_data(MAG_ADDR, MAG_R_ADDR, 8)
    mag_data = [0.0, 0.0, 0.0]
    for i in range(3):
        if i != 2:
            mag_data[i] = ((data[2 * i + 1] * 256) + (data[2 * i] & 0xF8)) / 8
            if mag_data[i] > 4095:
                mag_data[i] -= 8192
        else:
            mag_data[i] = ((data[2 * i + 1] * 256) + (data[2 * i] & 0xFE)) / 2
            if mag_data[i] > 16383:
                mag_data[i] -= 32768
    return mag_data

def calibrate_mag(raw_mag):
    """
    磁力計の補正を行う関数。
    MAX_X、MIN_X、MAX_Y、MIN_Yを使用して補正を適用する。
    """
    global MAX_X, MIN_X, MAX_Y, MIN_Y

    if None in (MAX_X, MIN_X, MAX_Y, MIN_Y) or MAX_X == MIN_X or MAX_Y == MIN_Y:
        print("キャリブレーションが未完了です。補正なしで値を返します。")
        return raw_mag[0], raw_mag[1]
    
    NOM_X = 2 * (raw_mag[0] - (MAX_X + MIN_X) / 2) / (MAX_X - MIN_X)  # X軸補正
    NOM_Y = 2 * (raw_mag[1] - (MAX_Y + MIN_Y) / 2) / (MAX_Y - MIN_Y)  # Y軸補正
    return NOM_X, NOM_Y

def calculate_heading(mag_data):
    NOM_X, NOM_Y = calibrate_mag(mag_data)  # 補正された磁力計データを取得
    heading = math.atan2(NOM_Y, NOM_X) * 180.0 / math.pi
    if heading < 0:
        heading += 360.0
    return heading

def calculate_acc_magnitude(acc_data):
    magnitude = math.sqrt(acc_data[0]**2 + acc_data[1]**2 + acc_data[2]**2)
    return magnitude

def create_csv_file():
    now_time = datetime.datetime.now()
    filename = 'test_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'
    with open(filename, 'a', newline="") as f:
        writer = csv.writer(f)
        writer.writerow(['Time', 'Acc_x', 'Acc_y', 'Acc_z', 'Acc_Magnitude', 'Gyro_x', 'Gyro_y', 'Gyro_z', 'Mag_x', 'Mag_y', 'Mag_z', 'Heading'])
    return filename

def SETPWM():
    # print('PWM',PLN.value, PLF.value, PRF.value, PRN.value)
    if abs(PLF.value - PLN.value) > PDEV * TSTEP:
        if PLF.value > PLN.value:
            PLN.value = PLN.value + PDEV * TSTEP
            #pin1.start(100 - PLN.value)
            pin1.start(PLN.value)
            print('pin1',100 - PLN.value)
            print(type(pin1))
        else:
            PLN.value = PLN.value - PDEV * TSTEP
            # pin1.start(100 - PLN.value)
            pin1.start(PLN.value)
       
    if abs(PRF.value - PRN.value) > PDEV * TSTEP:
        if PRF.value > PRN.value:
            PRN.value = PRN.value + PDEV * TSTEP
            #pin2.start(100 - PRN.value)
            pin2.start(PRN.value)
        else:
            PRN.value = PRN.value - PDEV * TSTEP
            # pin2.start(100 - PRN.value)
            pin2.start(PRN.value)

    if abs(PLF.value - PLN.value) <= PDEV * TSTEP:
        PLN.value = PLF.value
        pin1.start(PLN.value)        

    if abs(PRF.value - PRN.value) <= PDEV * TSTEP:
        PRN.value = PRF.value
        pin2.start(PRN.value)

def f(q, q1, TWE):
    rdata = ''
    """別プロセスで実行する関数。シリアルデータを読み取り、キューに入れる。"""
    while True:
        try:
            rdata = TWE.readline()
            LID_HEX=rdata[1:3]
            COM_HEX=rdata[3:5]      
            # if not rdata:
            #     print("データが受信されませんでした")
            #     continue
            if rdata[0] == 58 or COM_HEX == b"01":  # ':' または COM_HEXがb"01"
                if LID_HEX == b'08':  # pasokon
                    try:
                        hex_string = rdata[5:len(rdata) - 4]
                        result = ""
                        for i in range(0, len(hex_string), 2):
                            hex_char = hex_string[i:i+2]
                            character = chr(int(hex_char, 16))
                            result += character
                        if any(char in result for char in 'AWSUDLE'):
                            q1.put(result)
                        else:
                            q.put(result)
                            print(result)
                    except Exception as e:
                        print(f"HEX変換エラー: {e}")
                elif LID_HEX == b'05':  # 基地局データ
                    try:
                        received_data = rdata[5:len(rdata) - 4]
                        hex_string = received_data.decode('ascii')  # 必要に応じてエンコーディング確認
                        print("From05",hex_string)
                        byte_data = bytes.fromhex(hex_string)
                        if byte_data[0] != 36:  # '$'のASCIIコード
                            # print('error')
        
                            RTK_Ser.write(byte_data)  # 基地局データをRTK基板に送信
                    except Exception as e:
                        print(f"基地局データ処理エラー: {e}")
        except Exception as main_e:
            # print(f"メインループエラー: {main_e}")
            print("")   

#RTK用の変数
def cnvido(txt):
        try:
                RTK_NORTH.value=float(txt[:2])+(float(txt[2:])/60)
#               print(txt)
#               print(str(RTK_NORTH.value))
                return RTK_NORTH.value
        except ZeroDivisionError:
                print("0で割れません")

def cnvkeido(txt):
        try:
                RTK_EAST.value=float(txt[:3])+(float(txt[3:])/60)
#               print(txt)
#               print(str(RTK_EAST.value))
                return RTK_EAST.value
        except ZeroDivisionError:
                print("0で割れません")

#位置情報取得のためのサブプロセス。
def process1(RTK_UTC, RTK_EAST, RTK_NORTH, RTK_ANGLE, RTK_SPEED, RTK_FIX):
    while True:
        try:
            c = RTK_Ser.readline()
            if c[1:6] == b'GNRMC':
                sccc = c.decode()
                list_sccc = sccc.split(',')
                # list_sccc[5] の空チェック
                if len(list_sccc) > 5 and list_sccc[5]:
                    RTK_SPEED.value = float(list_sccc[5]) * 1.852
                else:
                    RTK_SPEED.value = 0.0  # デフォルト値を設定
            if c[1:6] == b'GNGGA':
                sc = c.decode()
                list_sc = sc.split(',')
                # list_sc 各フィールドの空チェック
                if len(list_sc) > 1 and list_sc[1]:
                    RTK_UTC.value = float(list_sc[1])
                if len(list_sc) > 4 and list_sc[4]:
                    RTK_EAST.value = float(list_sc[4])
                    cnvkeido(list_sc[4])
                if len(list_sc) > 2 and list_sc[2]:
                    RTK_NORTH.value = float(list_sc[2])
                    cnvido(list_sc[2])
                if len(list_sc) > 6 and list_sc[6]:
                    RTK_FIX.value = int(list_sc[6])
            if c[1:6] == b'GNVTG':
                scc = c.decode()
                list_scc = scc.split(',')
                if len(list_scc) > 1 and list_scc[1]:
                    RTK_ANGLE.value = float((list_scc[1])*-1)+360
                else:
                    RTK_ANGLE.value = 1000  # デフォルト値
        except ValueError as ve:
            print(f"ValueError: {ve}")
        except Exception as e:
            print(f"Unexpected error: {e}")
            
def STOP_NOW():
    global PLF
    global PLN
    global PRF
    global PRN
    PLF.value=0
    PLN.value=0
    PRF.value=0
    PRN.value=0
    pin1.start(0)  # デューティサイクル0で初期化
    pin2.start(0)  # デューティサイクル0で初期化    

def Caliblation_Of_Magnetic_Sencer(mode):
    global CMSS
    global end_time
    global received_data
    global MAX_X, MIN_X, MAX_Y, MIN_Y
    global PLF
    global PRF
    global speed 
    if CMSS ==1 :
        MAX_X, MIN_X, MAX_Y, MIN_Y = -float('inf'),float('inf'), -float('inf'), float('inf')
        start_time = datetime.now()
        speed = 100  #角度を多く取得するための減算済みのスピード変数
        end_time = start_time +timedelta(seconds=3)
        zenshin(speed, PLF, PRF)
        CMSS =2
    elif CMSS == 2:
       current_time = datetime.now()
       if current_time  < end_time  :
          pass
       else :
             CMSS = 3
             speed = 100  #角度を多く取得するための減算済みのスピード変数
             frac =0.4   #校正の時の回転基準
             migi(speed, frac, PLF, PRF)
             start_time = datetime.now()
             end_time = start_time +timedelta(seconds=5)
    elif CMSS == 3:
        current_time = datetime.now()
        if current_time  < end_time  :
            
            mag = mag_value()
            MAX_X = max(MAX_X, mag[0])
            MIN_X = min(MIN_X, mag[0])
            MAX_Y = max(MAX_Y, mag[1])
            MIN_Y = min(MIN_Y, mag[1])
        else:
            if mode == None:
                STOP_NOW()
            CMSS = 1
            received_data = mode

    print('CMSS',CMSS)
        
        
def culc_move_angle(lat1,lon1,lat2,lon2,heading):
    #move_angle 動くべき角度 +-180
    #dist_angle 現在地から目的地への角度  0-360
    #heading 機体の角度 360
    lat2 = RTK_NORTH.value
    lon2 = RTK_EAST.value
    lat_distance:float = (lat1 - lat2) * 111320  # 緯度差をメートルに変換
# 経度1度の距離を緯度のコサインで調整（経度差をメートルに変換）
    lon_distance:float = (lon1 - lon2) * 111320 * math.cos(math.radians((lat1 + lat2) / 2))
# 距離計算（ピタゴラスの定理）
    distance  = math.sqrt(lon_distance ** 2 + lat_distance ** 2)
    dist_Angle = math.degrees(math.atan2(-lon_distance,lat_distance))
    if dist_Angle < 0 :
        dist_Angle =dist_Angle+360

    # 結果を出力
    print(f"2点間の距離: {distance:.5f} メートル")
    print(f"方位角: {dist_Angle:.2f}°")
   
    Move_Angle = dist_Angle - heading
    # print(f'角度差:{Move_Angle:.2f}')
    if  Move_Angle > 180:
        Move_Angle = Move_Angle -360
    elif Move_Angle < -180:
        Move_Angle = Move_Angle +360
    return Move_Angle , distance , dist_Angle
       

def Auto_Function(lat1,lon1,lat2,lon2,heading):
    """
    RTKから取得した情報を基に現在地点と目的地点の距離と角度を計算する関数。
    """
    global received_data
    global NOM_X
    global NOM_Y
    global speed #読んで字のごとく
    global log  #TWELITE送信用アクション内容格納変数
    global log_counter  #動作安定目的用変数 
    if log_counter is None: #実質初期化
        log_counter=0
    #Rは角度指定としているため、Rの値を変こすることで判断角を変えることができる。
    R1 = 10
    R2 = 40
    R3 = 100
    print("自動操作を開始します")
    Move_Angle , distance , dist_Angle = culc_move_angle(lat1,lon1,lat2,lon2,heading)
    print('距離:',distance)
    print('機体角度 :',heading,'目的角度:',dist_Angle,'動くべき角度',Move_Angle)
    if distance < 2:
        speed = 55
    else :
        speed = 100
    if distance > 0.35 and log_counter == 0:  #1m以上のときに動作する
        #校正を行ったうえで不十分であると判断できる場合
        if NOM_X > 1.3 or NOM_Y > 1.3 or NOM_X < -1.3 or NOM_Y < -1.3:
            Caliblation_Of_Magnetic_Sencer('MJ') 
        if abs(Move_Angle) <R1:
            zenshin(speed, PLF, PRF)
            print('デバッグ用モニタ:前進')
            log='ST'  
        elif abs (Move_Angle)< R2:
            frac = 0.8
            if Move_Angle > 0:
                hidari(speed,frac, PLF, PRF) 
                print('デバッグ用モニタ:左R2') 
                log='L2'
            elif Move_Angle < 0:
                migi(speed,frac, PLF, PRF)
                print('デバッグ用モニタ:右R2') 
                log='R2'
        elif abs (Move_Angle)< R3:   
            frac = 0.4         
            if Move_Angle > 0:
                hidari(speed,frac, PLF, PRF) 
                print('デバッグ用モニタ:左R3') 
                log='L3'
            elif Move_Angle < 0:
                migi(speed,frac, PLF, PRF)
                print('デバッグ用モニタ:右R3') 
                log='R3'
        else:            
            frac = 0.3
            if Move_Angle > 0:
                hidari(speed,frac, PLF, PRF) 
                print('デバッグ用モニタ:左例外') 
                log='L:E'
            elif Move_Angle < 0:
                migi(speed,frac, PLF, PRF)
                print('デバッグ用モニタ:右例外') 
                log='R:E'
    else:
        stop(PLF, PRF)
        print('デバッグ用モニタ:停止')
        log= 'stop'
        log_counter = 1
        received_data = 'END'
    return log
                    
def Object_function():
    print('現在地点を登録します。')
    # 初期化
    lat1 : double= 0
    lon1 : double= 0

    # 緯度・経度が取得できるまでループ
    while lat1 == 0 or lon1 == 0:
        lat1 = RTK_NORTH.value  # multiprocessing.Value の値を取得
        lon1 = RTK_EAST.value
        if lat1 == 0 or lon1 == 0:
            print("データが取得できていません。再試行します...")
            sleep(0.5)  # 1秒待機して再試行
    # 取得できた座標を表示
    # print("登録座標: 緯度: {:.15f}, 経度: {:.15f},".format(lat1, lon1))  # 小数点以下15桁まで表示
    return lat1, lon1  
def write_log(message):
    """ログファイルに1行ずつ書き込み、即座にクローズ"""
    with open(log_filename, "a", encoding="utf-8") as log_file:
        log_file.write(f"{message}")
        log_file.flush()  # 即座にディスクに書き込
        
def first_Presher_():
    bus = init_i2c_bus(I2C_BUS_ID)
        # 初回の読み取りが有効になるまでリトライする
    first_pressure = 0
    for _ in range(10):
        first_pressure = read_pressure(bus, MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB)
        print(first_pressure)
        if first_pressure > 0:
            break
        time.sleep(1)  # リトライの間に1秒待機
        if first_pressure == 0:
            files.close()  # ファイルを閉じる
        raise RuntimeError("有効な気圧データが取得できませんでした")
    return first_pressure


def main():
    lat1 : double = 0.0
    lat2 : double = 0.0
    lon1 : double = 0.0
    lon2 : double = 0.0
    min_pressure:double = 0
    minpres =0.0
    maxpres = 0.0
    global dist_Angle
    global Move_Angle
    global log_counter
    global distance
    global NOM_X
    global NOM_Y
    global log
    global MAX_X, MIN_X, MAX_Y, MIN_Y
    bus = init_i2c_bus(I2C_BUS_ID)
    heading =0
    pretime =time.time()
    process = Process(target=process1,args=[RTK_UTC, RTK_EAST, RTK_NORTH, RTK_ANGLE, RTK_SPEED, RTK_FIX])
    process.start()
    # sum_distance=Process(target=args=[lon1,lon2,lat1,lat2,])
    p = Process(target=f, args=(q, q1, TWE))
    p.start()
    global received_data

    # received_data :str =None
    try:
        check_device(bus, MPL3115A2_ADDRESS, MPL3115A2_WHOAMI)
        configure_device(bus, MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1,
                         MPL3115A2_CTRL_REG1_SBYB | MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_BAR)
    except RuntimeError as e:
        print(e)
        files.close()  # ファイルを閉じる
        return
    first_pressure=first_Presher_()
    pressure = 0
    while True:
        pressure = read_pressure(bus, MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB)
        if pressure == 0:
            raise RuntimeError("有効な気圧データが取得できませんでした")
        start_time:datetime=time.time()
        elapsed_time:datetime = start_time - pretime
        pretime:datetime = start_time
        acc =  acc_value()
        # ジャイロスコープデータを取得
        gyro =  gyro_value()
        # 磁力計データを取得
        mag =  mag_value()
        # 方位角を計算
        heading= calculate_heading(mag)
        NOM_X, NOM_Y = calibrate_mag(mag)  # 補正後の磁力計データ
        if heading >0 :
            heading = 360-heading
        print( "現在角度:",heading)
        print('精度:',RTK_FIX.value)
        # 新しい入力を優先処理
        while not q.empty() :
            received_data = q.get()
            print("Received:", received_data)
        #現在地点登録
        if received_data == 'MT':
            lat1, lon1,  = Object_function()
            log_counter = None      #再度自動化可能にするための変数
            received_data = None    #モードリセット
            log = None              #C#表示用の変数をリセット
            #自動操作用
        elif received_data == 'MJ':
            # print(f"自動モード実行: lat1={lat1}, lon1={lon1}")
            if lat1 is not None and lon1 is not None:
                Move_Angle , distance , dist_Angle = culc_move_angle(lat1,lon1,lat2,lon2,heading)
                Auto_Function(lat1,lon1,lat2,lon2,heading)
            #位置登録がされていないとき
            else:
                print("位置データが不足しています。")
            #校正モード
        elif received_data == 'MC':
            Caliblation_Of_Magnetic_Sencer(None)
            #リモコン(手動操作)
        # elif received_data == 'MR':
        #     Remote_function()
            #校正と自動操作を一括で実行する
        elif received_data == 'Full':
            Caliblation_Of_Magnetic_Sencer('MJ')
        elif received_data == 'END':
            print("エラー防止")
        #気圧センサの一連の動作
        elif received_data == "First":
            received_data = None    #モードリセット
            first_pressure=first_Presher_()
            
        elif received_data == 'hPa':
                min_pressure = pressure  # 初期値として最初のpressureを設定
                print(f"MStartモード初回の気圧: {first_pressure:.2f} hPa 現在の気圧: {pressure:.2f} hPa 差: {first_pressure - pressure:.2f} hPa")
                if (first_pressure - pressure) >= 0.15:  # 1.5hpが15m 0.15hpが1.5m
                    print("1.5m以下になっているはず")
                    received_data = 'M30'
        elif received_data == 'M30':
            # 最小値の更新
            if pressure < min_pressure:
                min_pressure = pressure
            minpres, maxpres = dev10(pressure)
            print(f"M30モード現在の気圧: {pressure:.2f} hPa 最小値: {pressure:.2f} hpa")
            if pressure - min_pressure >= 0.15:
                received_data = 'M00'
        elif received_data == 'M00':
            minpres, maxpres = dev10(pressure)
            print(f"M00モード現在の気圧: {pressure:.2f} hPa 直近10個の最大値: {maxpres:.2f} hPa 直近10個の最小値: {minpres:.2f} hPa 差: {maxpres - minpres:.2f} hPa")
            if (maxpres - minpres) < 0.04:  # 判定を緩くした0.01 →0.04
                received_data = 'MOpen'
        elif received_data == 'MOpen':
            print("気圧が安定しました。サーボモーターを動かします。")
            move_servo(90)
            time.sleep(2)
            move_servo(0)
            time.sleep(2)
            move_servo(180)
            time.sleep(2)
            move_servo(0)
            time.sleep(2)
            received_data = "Full"
                    
        print(f"現在のモード: {received_data}")
        formatted_timestamp = datetime.now().strftime("time%Y-%m-%d %H:%M:%S")
        # message = "{},{},{},{},{},{},{},".format(RTK_UTC.value,RTK_NORTH.value, RTK_EAST.value,RTK_ANGLE.value,RTK_SPEED.value,RTK_FIX.value,heading)
        message = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format(
            str(formatted_timestamp),RTK_NORTH.value, RTK_EAST.value,RTK_SPEED.value,RTK_FIX.value,heading,dist_Angle,Move_Angle,lat1,lon1,
            received_data,distance,log,MAX_X,MIN_X,MAX_Y,MIN_Y,double(RTK_ANGLE.value),acc[0],acc[1],acc[2],
            gyro[0],gyro[1],gyro[2],mag[0],mag[1],mag[2],NOM_X,NOM_Y,PLF.value,PLN.value,PRF.value,PRN.value,elapsed_time,str(speed),pressure,first_pressure,min_pressure,minpres,maxpres)
        # print(Move_Angle)
        command = make_command(message)
        # print(message)
        TWE.write(command)
        write_log(message)
        print('現在地点:',RTK_NORTH.value,RTK_EAST.value)
        print('登録地点:',lat1,lon1)
        print(received_data)
        # TWEwrite((message + "\r\n").encode('utf-8'))
        # print('before set PWM')
        with open(fname, 'a') as log_f:
                log_f.write(message)
        if  lat1 != None and  lon1 != None:
            print("登録座標: 緯度: {:.15f}, 経度: {:.15f},".format(lat1, lon1))  # 小数点以下15桁まで表示
        # print("登録地点:",lat1,lon1)
        SETPWM()
        sleep(TSTEP)#停止が上手く効くように願った内容
#変換系のそーす。

def make_command(message):
    bytecode_list = string_to_bytecode_list(message)
    logical_id = 0x08
    command = 1
    packet = [logical_id, command] + bytecode_list
    checksum = calculate_2s_complement(packet)
    packet.append(checksum)
    return (":" + bytecode_list_to_hex_string(packet) + '\r\n').encode('utf-8')

def bytecode_list_to_hex_string(bytecode_list):
    return ''.join(f'{byte:02X}' for byte in bytecode_list)

def string_to_bytecode_list(input_string):
    return list(input_string.encode('utf-8'))

def calculate_2s_complement(bytecode_list):
    total_sum = sum(bytecode_list)
    lower_8_bits = total_sum % 256
    return (256 - lower_8_bits) % 256

# モーター操作関数
def zenshin(speed, PLF, PRF):
    PLF.value = speed
    PRF.value = speed
    
def stop(PLF, PRF):
    PLF.value = 0
    PRF.value = 0

def migi(speed, Frac, PLF, PRF):
    PLF.value = speed
    PRF.value = speed * Frac

def hidari(speed, Frac, PLF, PRF):
    PLF.value = speed * Frac
    PRF.value = speed

# def Test_Run(speed,frac,PLF,PRF):
#     PLF.value = speed * frac
#     PRF.value = speed
if __name__ == "__main__":
    RTK_UTC = Value('d',0)
    RTK_EAST = Value('d',0)
    RTK_NORTH = Value('d',0)
    RTK_ANGLE = Value('d',0)
    RTK_SPEED = Value('d',0)
    RTK_FIX = Value('i',0)
   
    bmx_setup()
    path = '/dev/ttyACM0'
    is_file = os.path.exists(path)
    while is_file==False:
            sleep(1)
    print("wait")
    # subprocess.Popen('/home/tt/RTKLIB/app/str2str/gcc/str2str -in ntrip://guest:guest@160.16.134.72:80/FUKUYAMANT -p 34.502872166 133.348872758 68.6904 -out serial://ttyACM0:115200', shell=True)
   
    #ここまでRTKの呼び出しに関係するプログラム。
    main()