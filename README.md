# CAR remote control form web<br>
consist of: STM32f407 board, ESP8266 board, 2 drivers motor LM298, Variable resistor 10k.
<br><br>
**Step 1.The control file named Car_control.ino is located in the ESP8266 folder:**<br>
    Edit the 2 variables "ssid" and "password" which are your wifi name and password.<br> 
    Connect your phone to that wifi.<br>
    Load code into ESP8266:<br>
       \t(Below is the address of the esp8266 server "192.168.180.177" in local, maybe your address will be different)<br>
        \tConnecting to WiFi...<br>
        \tConnecting to WiFi...<br>
        \tConnected to WiFi<br>
        \tIP Address: 192.168.180.177<br>
        \tHTTP server started<br><br> 
**Step 2. Load code into STM32f407 board**<br> <br> 
**Step 3. Access the IP address to remote control the vehicle**