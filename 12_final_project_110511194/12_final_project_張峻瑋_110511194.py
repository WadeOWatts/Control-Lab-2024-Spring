import cv2
import numpy as np
import serial
import time
from rplidar import RPLidar, RPLidarException
from math import floor
import threading



def write_read(x):                                      # Write instruciton to the Arduino board, and read info from it
    ser.write(bytes(str(x), 'utf-8')) 
    data = ser.readline().strip().decode('utf-8')       # 移除首尾空白並解碼為字串
    return int(data) if data else None                  # 將字符串轉換為整數，如果為空則返回 None



def lidar_scan_thread(lidar, scan_data, stop_event):    # Thread for lidar scanning, it must be another thread or it won't work
    try:
        for scan in lidar.iter_scans():
            if stop_event.is_set():
                break
            for (_, angle, distance) in scan:
                if distance > 0:                        # Check for valid distance value
                    scan_data[min([359, floor(angle)])] = distance # Put the data into scan_data array by angle order
    except RPLidarException as e:
        print(f"Exception while reading scan data: {e}")
        lidar.stop()                                    # Close the lidar if something wrong happen
        lidar.disconnect()
        lidar = RPLidar('/dev/ttyUSB1', timeout=3)
        time.sleep(1)



def measuring_distance(scan_data):                      # Criterions of front/left/right distances
    #global scan_data
    front = 0
    front_count = 0
    for i in range(10):                                 # Front distance is the average angle from -10 to 10 degrees
        if scan_data[i] > 0:
            front += scan_data[i]
            front_count += 1
    for i in range(350, 360):
        if scan_data[i] > 0:
            front += scan_data[i]
            front_count += 1
    if front_count != 0:
        front = front / front_count 
    else:
        front = 10000                                   # If all of value is 0, which is impossible, set from distance as 10000

    left = 0
    left_count = 0
    for i in range(260, 300):                           # Left distance is the average angle from 260 to 300 degrees
        if scan_data[i] > 0:
            left += scan_data[i]
            left_count += 1
    if left_count != 0:
        left = left / left_count
    else:
        left = 10000

    right = 0
    right_count = 0
    for i in range(60, 100):                            # Right distance is the average angle from 60 to 100 degrees
        if scan_data[i] > 0:
            right += scan_data[i]
            right_count += 1
    if right_count != 0:
        right = right / right_count
    else:
        right = 10000

    return front, right, left 



PORT_NAME = '/dev/ttyUSB1'                              # Define LiDAR
lidar = RPLidar(PORT_NAME, timeout=3)
scan_data = [0]*360                                     # Initialize scan_data array
stop_event = threading.Event()

cap = cv2.VideoCapture(0)                               # 打開影片

if not cap.isOpened():                                  # 確認影片成功打開
    print("Error: Could not open video file.")
    exit()

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)               #取得畫面長和寬
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print(f"Width: {width}")
print(f"Height : {height}")

path_stage = 1                                          # Initialize the stage in the LiDAR mode
max_distance = 0

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)) #侵蝕膨脹
vertices1 = np.array([[(0,250), (640, 250), (640, 480), (0, 480)]], dtype=np.int32) #選取霍夫找線範圍

min_slope = -0.4                                        #用於篩選線，把斜率在這之間的線去除
max_slope = 0.4
left_avg_x_at_y300 = None
right_avg_x_at_y300 = None

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)   # 初始化串口，根據你的Arduino串口號和波特率進行調整
time.sleep(1)

kp = 0.1                                                # PID 控制器參數
ki = 0
kd = 0.4 #0.08

sum_err = 0                                             # PID 控制器變量
last_err = 0
err = 0

offset = 350                                            # 當只有一條線時 定義車道中央為與線距離400的地方
point_at_y = 400                                        # 定義中央點的y座標



while True:
    ret, frame = cap.read()
    
    if not ret:                                         # 確認是否成功讀取影片
        print("Error: Failed to read frame.")
        break

    #---------------------------------------------------------------------------------------------------------------
    # 判定三角形的影像處理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)      # 將圖像轉換為灰階
    edges = cv2.Canny(gray, 30, 200)                    # 進行 Canny 邊緣檢測
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # 輪廓檢測
    #---------------------------------------------------------------------------------------------------------------

    mask = np.zeros_like(frame[:, :, 0])                                        # 建立遮罩
    cv2.fillPoly(mask, vertices1, 255)
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)                     # 應用遮罩

    #二值化播放
    gray_frame = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)                 #灰階
    _, binary_frame = cv2.threshold(gray_frame, 100, 255, cv2.THRESH_BINARY)    #二值化

    # 顯示侵蝕膨脹畫面
    eroded_frame = cv2.erode(binary_frame, kernel, iterations=1)                #侵蝕
    dilated_frame = cv2.dilate(eroded_frame, kernel, iterations=1)              #膨脹

    # Canny 邊緣檢測
    edges = cv2.Canny(gray_frame, 25, 200, apertureSize=3)

    # 使用霍夫變換找線
    lines = cv2.HoughLines(edges,0.8,np.pi/180,80)    
    # 前面的數字是分辨率，數字越大精度越低，運算越快
    # 後面的數字是檢測閥值，大於閥值才會辨認成功，值越小越容易辨認成功，但也越容易有雜訊

    #---------------------------------------------------------------------------------------------------------------

    # 存取左右車道線
    left_lines = []
    right_lines = []

    # 如果找到線，則繪製
    if lines is not None:
        for rho, theta in lines[:, 0]:
            # 計算斜率
            if np.tan(theta)!=0:
                slope = -1 / np.tan(theta)
            # 確認斜率範圍
            if slope < min_slope or slope > max_slope:
                # 根據斜率將直線分類為左右車道線
                if slope < 0:
                    left_lines.append((rho, theta))
                else:
                    right_lines.append((rho, theta))

    #---------------------------------------------------------------------------------------------------------------

    # 繪製所有左右車道線
    line_image = np.zeros_like(frame)
    if left_lines:
        for rho, theta in left_lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    if right_lines:
        for rho, theta in right_lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # 將原本畫面加上左右車道線
    result_frame = cv2.addWeighted(frame, 0.8, line_image, 1, 0)

    #---------------------------------------------------------------------------------------------------------------

    # 繪製左右車道平均線
    line_image = np.zeros_like(frame)
    if left_lines:
        left_mean_line = np.mean(np.array(left_lines), axis=0)
        rho, theta = left_mean_line
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        left_avg_x_at_y300 = int((rho - point_at_y * np.sin(theta)) / np.cos(theta)) # 計算左平均線在 y=300 時的 x 座標
    if right_lines:
        right_mean_line = np.mean(np.array(right_lines), axis=0)
        rho, theta = right_mean_line
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        right_avg_x_at_y300 = int((rho - point_at_y * np.sin(theta)) / np.cos(theta)) # 計算右平均線在 y=300 時的 x 座標

    #---------------------------------------------------------------------------------------------------------------

    # 計算兩條平均線的中點
    midpoint_x = None
    if left_lines and right_lines:
        left_mean_line = np.mean(np.array(left_lines), axis=0)
        right_mean_line = np.mean(np.array(right_lines), axis=0)

        rho_left, theta_left = left_mean_line
        rho_right, theta_right = right_mean_line

        # 根據 y = 400 的位置計算兩條平均線的 x 座標
        y_target = point_at_y
        x_left = (rho_left - y_target * np.sin(theta_left)) / np.cos(theta_left)
        x_right = (rho_right - y_target * np.sin(theta_right)) / np.cos(theta_right)

        # 計算兩條平均線的中點
        midpoint_x = int((x_left + x_right) / 2)

    # 繪製紅點（螢幕中央點）
    cv2.circle(line_image, (320, point_at_y), 10, (0, 0, 255), -1)  

    # 繪製藍點（車道中央點）
    # 檢查兩條平均線是否都存在，如果其中一條不存在，則添加單側點
    if midpoint_x is None:                                          # 有一條不存在
        if left_lines and not right_lines:                          # 只有左平均線存在時
            cv2.circle(line_image, (left_avg_x_at_y300 + offset, point_at_y), 10, ( 255, 255, 0), -1)
            err = left_avg_x_at_y300 + offset - 320
        if right_lines and not left_lines:                          # 只有右平均線存在時
            cv2.circle(line_image, (right_avg_x_at_y300 - offset, point_at_y), 10, (255, 255, 0), -1)
            err = right_avg_x_at_y300 - offset - 320
    else:                                                           # 都存在
        cv2.circle(line_image, (midpoint_x, point_at_y), 10, ( 255, 255, 0), -1)
        err = midpoint_x - 320

    #---------------------------------------------------------------------------------------------------------------
    
    # 計算 PID 控制輸出
    u = kp * err + ki * sum_err + kd * (err - last_err)
    u = int(u)

    # 更新 PID 控制器變量
    sum_err += err
    last_err = err

    received_data = write_read(u)
    if received_data is not None:
        print(received_data)

    if received_data == 1234:                       # 將進入 LiDAR mode
        break
    #---------------------------------------------------------------------------------------------------------------
    #三角形識別
    
    # 進行形狀檢測和重心計算
    for contour in contours:
        # 計算逼近多邊形
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 如果逼近的多邊形有3個頂點，且面積大於3000，則認為是三角形
        if len(approx) == 3 and cv2.contourArea(contour) > 3000:
            # 獲取三個頂點的座標
            vertices = np.squeeze(approx)

            #透過找三個頂點的x座標 判斷朝左或朝右
            a= abs(vertices[0][0] - vertices[1][0]) # a b c 是兩兩頂點的距離
            b= abs(vertices[1][0] - vertices[2][0])
            c= abs(vertices[2][0] - vertices[0][0])
            if a < b and a < c:                     #特殊點是 vertices[2][0]
                if vertices[2][0]>vertices[1][0] and vertices[2][0]>vertices[0][0]:     #右
                    received_data = write_read(9000)
                    if received_data is not None:
                        print(received_data)
                elif  vertices[2][0]<vertices[1][0] and vertices[2][0]<vertices[0][0]:  #左
                    received_data = write_read(8000)
                    if received_data is not None:
                        print(received_data)
            if b < a and b < c:                     #特殊點是vertices[0][0]
                if vertices[0][0]>vertices[1][0] and vertices[0][0]>vertices[2][0]:     #右
                    received_data = write_read(9000)
                    if received_data is not None:
                        print(received_data)
                elif  vertices[0][0]<vertices[1][0] and vertices[0][0]<vertices[2][0]:  #左
                    received_data = write_read(8000)
                    if received_data is not None:
                        print(received_data)
            if c < a and c < b:                     #特殊點是vertices[1][0]
                if vertices[1][0]>vertices[0][0] and vertices[1][0]>vertices[2][0]:     #右
                    received_data = write_read(9000)
                    if received_data is not None:
                        print(received_data)
                elif  vertices[1][0]<vertices[0][0] and vertices[1][0]<vertices[2][0]:  #左
                    received_data = write_read(8000)
                    if received_data is not None:
                        print(received_data)

    #---------------------------------------------------------------------------------------------------------------

    # 將原本畫面加上平均線和點
    result_frame1 = cv2.addWeighted(frame, 1, line_image, 1, 0)

    if received_data is None:
        received_data = 0

    # err:誤差 u:PID算出結果 L:左輪轉速 R:右輪轉速
    text2=str(u)
    cv2.putText(result_frame1, text2, (90, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text3 = "err:"
    cv2.putText(result_frame1, text3, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text4 = "u:"
    cv2.putText(result_frame1, text4, (55, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text5 = "L:"
    cv2.putText(result_frame1, text5, (450, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text6 = "R:"
    cv2.putText(result_frame1, text6, (450, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text7 = str(30+received_data)
    cv2.putText(result_frame1, text7, (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text8 = str(30-received_data)
    cv2.putText(result_frame1, text8, (500, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text1 = str(err)
    cv2.putText(result_frame1, text1, (90, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA) # 文字輸出在(90,50)

    # 顯示帶有線條和紅點的畫面
    cv2.imshow('average line', result_frame1)

    #---------------------------------------------------------------------------------------------------------------
    # 等待按按鍵
    key = cv2.waitKey(25) # 原速是25

    # 按q退出
    if key == ord('q'):
        break
    elif key == ord('w'):
        ser.write(bytes(str(1000), 'utf-8')) 
    elif key == ord('a'):
        ser.write(bytes(str(2000), 'utf-8')) 
    elif key == ord('s'):
        ser.write(bytes(str(3000), 'utf-8')) 
    elif key == ord('d'):
        ser.write(bytes(str(4000), 'utf-8')) 
    elif key == ord('e'):
        ser.write(bytes(str(5000), 'utf-8')) 
    elif key == ord('f'):
        ser.write(bytes(str(6000), 'utf-8'))


for i in range(30):                                                     # Do 30 times PID control before going into LiDAR mode
    front_dis, right_dist, left_dist = measuring_distance(scan_data) 
    ret, frame = cap.read()
    
    # 確認是否成功讀取影片
    if not ret:
        print("Error: Failed to read frame.")
        break

    #---------------------------------------------------------------------------------------------------------------
    # 判定三角形的影像處理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)              # 將圖像轉換為灰階
    edges = cv2.Canny(gray, 30, 200)                            # 進行 Canny 邊緣檢測
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    # 輪廓檢測
    #---------------------------------------------------------------------------------------------------------------

    mask = np.zeros_like(frame[:, :, 0])                                # 創建遮罩
    cv2.fillPoly(mask, vertices1, 255)

    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)             # 應用遮罩
    #二值化播放
    gray_frame = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)         # 灰階
    _, binary_frame = cv2.threshold(gray_frame, 100, 255, cv2.THRESH_BINARY) # 二值化

    # 顯示侵蝕膨脹畫面
    eroded_frame = cv2.erode(binary_frame, kernel, iterations=1)        # 侵蝕
    dilated_frame = cv2.dilate(eroded_frame, kernel, iterations=1)      # 膨脹

    # Canny 邊緣檢測
    edges = cv2.Canny(gray_frame, 25, 200, apertureSize=3)

    # 使用霍夫變換找線
    lines = cv2.HoughLines(edges,0.8,np.pi/180,80)    
    # 前面的數字是分辨率，數字越大精度越低，運算越快
    # 後面的數字是檢測閥值，大於閥值才會辨認成功，值越小越容易辨認成功，但也越容易有雜訊

    #---------------------------------------------------------------------------------------------------------------

    # 存取左右車道線
    left_lines = []
    right_lines = []

    # 如果找到線，則繪製
    if lines is not None:
        for rho, theta in lines[:, 0]:
            # 計算斜率
            if np.tan(theta)!=0:
                slope = -1 / np.tan(theta)
            # 確認斜率範圍
            if slope < min_slope or slope > max_slope:
                # 根據斜率將直線分類為左右車道線
                if slope < 0:
                    left_lines.append((rho, theta))
                else:
                    right_lines.append((rho, theta))

    #---------------------------------------------------------------------------------------------------------------

    # 繪製所有左右車道線
    line_image = np.zeros_like(frame)
    if left_lines:
        for rho, theta in left_lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    if right_lines:
        for rho, theta in right_lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # 將原本畫面加上左右車道線
    result_frame = cv2.addWeighted(frame, 0.8, line_image, 1, 0)

    #---------------------------------------------------------------------------------------------------------------

    # 繪製左右車道平均線
    line_image = np.zeros_like(frame)
    if left_lines:
        left_mean_line = np.mean(np.array(left_lines), axis=0)
        rho, theta = left_mean_line
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        left_avg_x_at_y300 = int((rho - point_at_y * np.sin(theta)) / np.cos(theta))    # 計算左平均線在 y=300 時的 x 座標
    if right_lines:
        right_mean_line = np.mean(np.array(right_lines), axis=0)
        rho, theta = right_mean_line
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        right_avg_x_at_y300 = int((rho - point_at_y * np.sin(theta)) / np.cos(theta))   # 計算右平均線在 y=300 時的 x 座標

    #---------------------------------------------------------------------------------------------------------------

    # 計算兩條平均線的中點
    midpoint_x = None
    if left_lines and right_lines:
        left_mean_line = np.mean(np.array(left_lines), axis=0)
        right_mean_line = np.mean(np.array(right_lines), axis=0)

        rho_left, theta_left = left_mean_line
        rho_right, theta_right = right_mean_line

        # 根據 y = 400 的位置計算兩條平均線的 x 座標
        y_target = point_at_y
        x_left = (rho_left - y_target * np.sin(theta_left)) / np.cos(theta_left)
        x_right = (rho_right - y_target * np.sin(theta_right)) / np.cos(theta_right)

        # 計算兩條平均線的中點
        midpoint_x = int((x_left + x_right) / 2)

    # 繪製紅點（螢幕中央點）
    cv2.circle(line_image, (320, point_at_y), 10, (0, 0, 255), -1)  

    # 繪製藍點（車道中央點）
    # 檢查兩條平均線是否都存在，如果其中一條不存在，則添加單側點
    if midpoint_x is None:                                  # 有一條不存在
        if left_lines and not right_lines:                  # 只有左平均線存在時
            cv2.circle(line_image, (left_avg_x_at_y300 + offset, point_at_y), 10, ( 255, 255, 0), -1)
            err=left_avg_x_at_y300 + offset - 320
        if right_lines and not left_lines:                  # 只有右平均線存在時
            cv2.circle(line_image, (right_avg_x_at_y300 - offset, point_at_y), 10, (255, 255, 0), -1)
            err=right_avg_x_at_y300 - offset - 320
    else:                                                   # 都存在
        cv2.circle(line_image, (midpoint_x, point_at_y), 10, ( 255, 255, 0), -1)
        err=midpoint_x - 320

    #---------------------------------------------------------------------------------------------------------------
    
    # 計算 PID 控制輸出
    u = kp * err + ki * sum_err + kd * (err - last_err)
    u=int(u)

    # 更新 PID 控制器變量
    sum_err += err
    last_err = err

    received_data = write_read(u)
    if received_data is not None:
        print(received_data)

    #---------------------------------------------------------------------------------------------------------------
    #三角形識別
    
    # 进行形状检测和重心计算
    for contour in contours:
        # 計算逼近多邊形
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 如果逼近的多邊形有3個頂點 且面積大於3000，則認為是三角形
        if len(approx) == 3 and cv2.contourArea(contour) > 3000:
            # 獲取三個頂点的座標
            vertices = np.squeeze(approx)

            # 透過找三個頂點的x座標 判斷朝左或朝右
            a= abs(vertices[0][0] - vertices[1][0])                                     # a b c 是兩兩頂點的距離
            b= abs(vertices[1][0] - vertices[2][0])
            c= abs(vertices[2][0] - vertices[0][0])
            if a < b and a < c:                                                         #特殊點是vertices[2][0]
                if vertices[2][0]>vertices[1][0] and vertices[2][0]>vertices[0][0]:     #右
                    received_data = write_read(9000)
                    if received_data is not None:
                        print(received_data)
                elif  vertices[2][0]<vertices[1][0] and vertices[2][0]<vertices[0][0]:  #左
                    received_data = write_read(8000)
                    if received_data is not None:
                        print(received_data)
            if b < a and b < c:                                                         #特殊點是vertices[0][0]
                if vertices[0][0]>vertices[1][0] and vertices[0][0]>vertices[2][0]:     #右
                    received_data = write_read(9000)
                    if received_data is not None:
                        print(received_data)
                elif  vertices[0][0]<vertices[1][0] and vertices[0][0]<vertices[2][0]:  #左
                    received_data = write_read(8000)
                    if received_data is not None:
                        print(received_data)
            if c < a and c < b:                                                         #特殊點是vertices[1][0]
                if vertices[1][0]>vertices[0][0] and vertices[1][0]>vertices[2][0]:     #右
                    received_data = write_read(9000)
                    if received_data is not None:
                        print(received_data)
                elif  vertices[1][0]<vertices[0][0] and vertices[1][0]<vertices[2][0]:  #左
                    received_data = write_read(8000)
                    if received_data is not None:
                        print(received_data)

    #---------------------------------------------------------------------------------------------------------------

    # 將原本畫面加上平均線和點
    result_frame1 = cv2.addWeighted(frame, 1, line_image, 1, 0)

    if received_data is None:
        received_data = 0

    # err:誤差 u:PID算出結果 L:左輪轉速 R:右輪轉速
    text2=str(u)
    cv2.putText(result_frame1, text2, (90, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text3 = "err:"
    cv2.putText(result_frame1, text3, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text4 = "u:"
    cv2.putText(result_frame1, text4, (55, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text5 = "L:"
    cv2.putText(result_frame1, text5, (450, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text6 = "R:"
    cv2.putText(result_frame1, text6, (450, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text7 = str(30+received_data)
    cv2.putText(result_frame1, text7, (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text8 = str(30-received_data)
    cv2.putText(result_frame1, text8, (500, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    text1 = str(err)
    cv2.putText(result_frame1, text1, (90, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA) # 文字輸出在(90,50)

    # 顯示帶有線條和紅點的畫面
    cv2.imshow('average line', result_frame1)


# 釋放影片資源並關閉
cap.release()
cv2.destroyAllWindows()
received_data = write_read(5000)



try:
    if lidar:
        lidar_thread = threading.Thread(target=lidar_scan_thread, args=(lidar, scan_data, stop_event))
        lidar_thread.start()                    # Start the thread of LiDAR scanning
        received_data = write_read(5500)        # Slightly turn left
        front_dis = 10000                       # Assume no obstacles in the front
        Left_Turn = False                       # Didn't turn left enough before entering stage 1
        while True: 
            while path_stage == 1:
                front_dis, right_dist, left_dist = measuring_distance(scan_data) # Measuring distances
                if Left_Turn == False:
                    if right_dist > 300:        # If there's a wall on the right
                        Left_Turn = True        # Then don't need to turn left
                    else:
                        received_data = write_read(7000)
                        if received_data == 7000:
                            Left_Turn = True
                        if received_data is not None:
                            print(received_data)
                elif Left_Turn == True and front_dis >= 500:    # If it is far away from the wall in the front
                    received_data = write_read(1000)            # Keep moving
                    if left_dist < 300:                         # If too close to the left
                        received_data = write_read(4000)        # Turn right
                        if received_data is not None:
                            print(received_data)
                    elif right_dist < 300:                      # If too close to the right
                        received_data = write_read(2000)        # Turn left
                        if received_data is not None:
                            print(received_data)
                elif Left_Turn == True and front_dis < 500:     # If close enough to the wall in the front
                    path_stage = 2                              # Enter stage 2
                    received_data = write_read(7000)            # Turn left
                    if received_data is not None:
                        print(received_data)
                    print("change path stage to ", path_stage)
                    break                        
            while path_stage == 2:
                front_dis, right_dist, left_dist = measuring_distance(scan_data)
                if front_dis >= 400:                            # If it is far away from the wall in the front
                    received_data = write_read(1000)
                    if received_data is not None:
                        print(received_data)
                    if left_dist > 1000:                        # If it has much space in the left
                        if right_dist < 400:                    # Keep some certain spce with the wall in the right
                            received_data = write_read(2000)
                            if received_data is not None:
                                print(received_data)
                        elif right_dist > 500:
                            received_data = write_read(4000)
                            if received_data is not None:
                                print(received_data)
                    else:                                       # If too close to the left
                        if left_dist < 300:
                            received_data = write_read(4000)
                            if received_data is not None:
                                print(received_data)
                        elif right_dist < 300:
                            received_data = write_read(2000)
                            if received_data is not None:
                                print(received_data)
                        elif right_dist > 800:
                            received_data = write_read(4000)
                            if received_data is not None:
                                print(received_data)
                else:
                    path_stage = 3                              # If there's a wall in the front, which means it is in the corner, the finish point
                    received_data = write_read(5000)            # Then stop.
                    if received_data is not None:       
                        print(received_data)
                    break
            while path_stage == 3:                              # Stop
                received_data = write_read(5000)
                if received_data is not None:
                    print(received_data)
                
except KeyboardInterrupt:
    print("Received interrupt signal, stopping program...")
finally:
    stop_event.set()
    if lidar:
        lidar.stop()
        lidar.disconnect()
    if 'lidar_thread' in locals():
        lidar_thread.join()