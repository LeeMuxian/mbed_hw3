# mbed_hw3

1.  
我用兩個大的function(在python裡用RPC來呼叫這兩個function)來分別代表那兩個mode。  
   
(a) 第一個大函式是gesture_UI，這個函式代表的是gesture UI mode，在這個函式裡面我用一個thread去執行一個叫做threshold_select的小函式，在threshold_select裡面主要在根據lab8做的手勢去改變threshold angle，
threshold angle預設是30度(也就是從30度開始往上增加)，每做一次lab8的手勢就增加5度，直到60度，到60度之後繼續做手勢的話會跳回30度重新增加。當我按下user button(也就是select comfirm)的時候，我的mbed會用
wifi和MQTT傳送包含threshold angle的訊息給我的PC，當PC收到這個訊息之後會回印/stop_mode/run(第三個RPC function call，用來終止現在這個模式用的)給screen，然後我的mbed就會跳出gesture UI mode。  
   
(b) 第二個大函式是angle_det，這個函式代表的是angle detection mode，在這個函式裡也一樣用一個thread去執行一個叫做thread_angle的小函式，thread_angle這個小函式是在讀取三軸加速器的數值，並且如果超過
threshold angle的話就會用wifi和MQTT傳送一段包含現在傾斜角度的訊息給PC，當PC收到10個這種訊息之後也一樣會回印/stop_mode/run給screen，然後我的mbed就會跳出angle detection mode。   
    
(c) 在計算角度是否超過threshold angle的函式叫做calculate_angle，我是用現在三軸加速器的這個向量和reference angle這個向量去做內積，這樣就可以得到兩向量夾角的cos值，最後再用arccos function去得到角度。     
    
        
        
2.
uLCD上顯示的threshold angle:  
![](https://i.imgur.com/IHtPodV.png)

python收到threshold angle的訊息:  
![](https://i.imgur.com/DekgzNS.png)

mbed發送threshold angle的訊息:  
![](https://i.imgur.com/3PAjbqI.png)

uLCD上顯示的tilt angle:  
![](https://i.imgur.com/sl2wOeF.png)

python收到tilt angle超過threshold angle的訊息:  
![](https://i.imgur.com/W4d62c5.png)

mbed發送tilt angle超過threshold angle的訊息:  
![](https://i.imgur.com/HYq3HKF.png)
