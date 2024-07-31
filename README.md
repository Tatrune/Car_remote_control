CAR remote control form web
consist of: STM32f407 board, ESP8266 board, 2 drivers motor LM298, Variable resistor 10k.

Step 1.The control file named Car_control.ino is located in the ESP8266 folder:
    Edit the 2 variables "ssid" and "password" which are your wifi name and password. 
    Connect your phone to that wifi.
    Load code into ESP8266:
        (Below is the address of the esp8266 server "192.168.180.177" in local, maybe your address will be different)
        Connecting to WiFi...
        Connecting to WiFi...
        Connected to WiFi
        IP Address: 192.168.180.177
        HTTP server started

Step 2. Load code into STM32f407 board

Step 3. Access the IP address to remote control the vehicle