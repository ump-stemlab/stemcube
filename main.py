from machine import Pin, I2C, UART
from ssd1306 import SSD1306_I2C
from mpu6050 import init_mpu6050, get_mpu6050_data
import bme280
import ads1x15
import utime

# OLED parameters
height = 32
width = 128

# Initiate modules
i2c=I2C(1,sda=Pin(2), scl=Pin(3), freq=400000)
oled = SSD1306_I2C(width, height, i2c)
init_mpu6050(i2c)
bme = bme280.BME280(i2c=i2c)
HC12 = UART(1, 9600)
UART1 = UART(0,9600)

adc_addr = 0x48
gain = 1
rate = 4
adc = ads1x15.ADS1115(i2c,adc_addr,gain)

# OLED functions
def write_oled(text, align, pos):
    font = 7
    numchar = len(text)*font
    if align.upper() == "R": distant = 0
    elif align.upper() == "M": distant = int((width-numchar)/2 - 5)
    elif align.upper() == "L": distant = ((width-numchar) - 7)
    else : dis = 0
    
    if pos == 1: position = 0
    elif pos == 2: position = int (height/4)
    elif pos == 3: position = int (height/2)
    elif pos == 4: position = int (height*3/4)
    else: position = 0
    
    oled.text(text,distant,position)

def display_oled(interval):
    oled.show()
    utime.sleep(interval)
    
delay_interval = 0.3
def write_oled_menu(text11,text12,text13,
                    text21,text22,text23,
                    text31,text32,text33,
                    text41,text42,text43):
    global delay_interval
    oled.fill_rect(0, 0, width, height, 0)
    
    write_oled(text11,"R",1)
    write_oled(text12,"M",1)
    write_oled(text13,"L",1)
    
    write_oled(text21,"R",2)
    write_oled(text22,"M",2)
    write_oled(text23,"L",2)
    
    write_oled(text31,"R",3)
    write_oled(text32,"M",3)
    write_oled(text33,"L",3)
    
    write_oled(text41,"R",4)
    write_oled(text42,"M",4)
    write_oled(text43,"L",4)
    
    display_oled(delay_interval)

# Physical pin mapping
pin_dict = {
    "pb1" : 18,
    "pb2" : 17,
    "pb3" : 16,
    "led1" : 14,
    "led2" : 13,
    "led3" : 12,
    "toggle" : 15,
    "buzzer" : 11
    }

# Set pins modes
pb1 = Pin(pin_dict["pb1"], Pin.IN)
pb2 = Pin(pin_dict["pb2"], Pin.IN)
pb3 = Pin(pin_dict["pb3"], Pin.IN)
toggle = Pin(pin_dict["toggle"], Pin.IN)

led1 = Pin(pin_dict["led1"],Pin.OUT)
led2 = Pin(pin_dict["led2"],Pin.OUT)
led3 = Pin(pin_dict["led3"],Pin.OUT)
buzzer = Pin(pin_dict["buzzer"], Pin.OUT)

pb1_counter, pb2_counter, pb3_counter = 0, 0 ,0
old_input_pb1, old_input_pb2, old_input_pb3 = 0,0,0

def check_pb_event():
    global pb1_counter, pb2_counter, pb3_counter
    global old_input_pb1, old_input_pb2, old_input_pb3
    global i2c
    
    wait_interval = 0.00
    
    input_pb1 = pb1.value()
    input_pb2 = pb2.value()
    input_pb3 = pb3.value()
    
    if input_pb1 == 1 and old_input_pb1 == 0:
        pb1_counter = pb1_counter + 1
        utime.sleep(wait_interval)
        return 1
    
    if input_pb2 == 1 and old_input_pb2 == 0:
        pb2_counter = pb2_counter + 1
        utime.sleep(wait_interval)
        return 2
    
    if input_pb3 == 1 and old_input_pb3 == 0:
        pb3_counter = pb3_counter + 1
        utime.sleep(wait_interval)
        return 3
    
    old_input_pb1 = input_pb1
    old_input_pb2 = input_pb2
    old_input_pb3 = input_pb3
     
current_state = 1
led_counter = 0
stateRed, stateYellow, stateGreen = 0, 0, 0
stateBuzzer = 0
uart_count = 0

def statemenu(input_state):
    global current_state
    global led_counter
    global stateRed, stateYellow, stateGreen
    global stateBuzzer
    global uart_count
    global delay_interval
    
    current_state = input_state
    
    data_display = "{:.2f}"
    
    #Check for pushbutton presses
    input = check_pb_event()
    
    #LED state
    if current_state == 1:
        if input == 2:
            led_counter = led_counter + 1
            
        if led_counter == 0 : 
            stateRed = stateYellow = stateGreen = 0
        elif led_counter == 1 :
            stateRed = 1
            stateYellow = stateGreen = 0
        elif led_counter == 2 :
            stateRed = stateYellow = 1
            stateGreen = 0
        elif led_counter == 3 :
            stateRed = stateYellow = stateGreen = 1
        # Overflow
        if led_counter > 3 : led_counter = 0
        
        # Output according to state
        led1.value(stateRed)
        led2.value(stateYellow)
        led3.value(stateGreen)
        
        if stateRed == 1: display_stateRed = "On"
        else: display_stateRed = "Off"

        if stateYellow == 1: display_stateYellow = "On"
        else:display_stateYellow = "Off"
        
        if stateGreen == 1: display_stateGreen = "On"
        else: display_stateGreen = "Off"
        # Display
        write_oled_menu(text11 = "", 					text12 = "LED", 				text13 = "",
                        text21 = "Red", 				text22 = "Yellow", 				text23 = "Green",
                        text31 = display_stateRed, 		text32 = display_stateYellow,	text33 = display_stateGreen,
                        text41 = "Back", 				text42 = "Set", 				text43 = "Next")
    # Buzzer state
    if current_state == 2:
        
        stateBuzzer = toggle.value()
        if stateBuzzer == 1: display_stateBuzzer = "On"
        else: display_stateBuzzer = "Off"
            
        write_oled_menu(text11 = "", 					text12 = "Buzzer", 				text13 = "",
                        text21 = "", 					text22 = display_stateBuzzer, 	text23 = "",
                        text31 = "", 					text32 = "",					text33 = "",
                        text41 = "Back", 				text42 = "", 				text43 = "Next")
        
    # Gyroscope state
    if current_state == 3:
        data = get_mpu6050_data(i2c)
        
        temp = data_display.format(data['temp'])
        accel_x = data_display.format(data['accel']['x'])
        accel_y = data_display.format(data['accel']['y'])
        accel_z = data_display.format(data['accel']['z'])
        
        gyro_x = data_display.format(data['gyro']['x'])
        gyro_y = data_display.format(data['gyro']['y'])
        gyro_z = data_display.format(data['gyro']['z'])
        
        write_oled_menu(text11 = "Accel", 					text12 = "", 				text13 = "Gyro",
                        text21 = accel_x, 					text22 = " X", 				text23 = gyro_x,
                        text31 = accel_y, 					text32 = " Y",				text33 = gyro_y,
                        text41 = accel_z, 					text42 = " Z", 				text43 = gyro_z)
    # BME280 state
    if current_state == 4:
        #Temp [0], Pressure [1], Humidity [2]
        values = bme.values
        
        write_oled_menu(text11 = "", 					text12 = "BME280", 						text13 = "",
                        text21 = "", 					text22 = str(values[0]), 				text23 = "",
                        text31 = "", 					text32 = str(values[1]),				text33 = "",
                        text41 = "", 					text42 = str(values[2]), 				text43 = "")

    # MQ3 Gas sensor
    if current_state == 5:
        
        #Analog sample gas sensor
        sample0 = adc.read(rate,0)
        voltage0 = adc.raw_to_v(sample0)
        display_voltage0 = data_display.format(voltage0) + " V"
        
        buzzer_enable = toggle.value()
        
        #! Do not use digital input, it goes up to 5V, may damage 3V3 hardware
        #Digital input gas sensor
        if(voltage0>2.70) :
            gas_display = "Gas Detected"
            if buzzer_enable: buzzer.value(1)
        else :
            gas_display = "No Gas"
            if buzzer_enable : buzzer.value(0)
            
        
        write_oled_menu(text11 = "", 					text12 = "MQ3", 				text13 = "",
                        text21 = "", 					text22 = gas_display, 			text23 = "",
                        text31 = "", 					text32 = display_voltage0,		text33 = "",
                        text41 = "Back", 				text42 = "", 					text43 = "Next")
        
    # HC12 Transceiver
    if current_state == 6:
        #Transmit if toggle on
        if toggle.value():
            uart_count = uart_count + 1
            
            HC12_state = "Transmitting"
            send = str(uart_count) + "\n"
            HC12.write(send.encode())
            display_HC12 = str(uart_count)
            
            utime.sleep(1 - delay_interval)
    
        #Receive if toggle off
        else:
            HC12_state = "Receiving"
            display_HC12 = ""
            
            # Test using UART1 to send to UART0 (HC12 terminal)
#             uart_count = uart_count + 1
#             test_string = "Hi " + str(uart_count)
#             UART1.write(test_string.encode())
#             utime.sleep(0.1)
            if HC12.any():
                read_HC12 = HC12.readline()
                if len(read_HC12) > 0:
                    display_HC12 = str(read_HC12.decode())
        
        write_oled_menu(text11 = "", 					text12 = "HC12", 				text13 = "",
                        text21 = "", 					text22 = HC12_state, 			text23 = "",
                        text31 = "", 					text32 = display_HC12,			text33 = "",
                        text41 = "Back", 				text42 = "", 					text43 = "Next")
            
        
    # ADC state
    if current_state == 7:
        
        sample1 = adc.read(rate,1)
        sample2 = adc.read(rate,2)
        sample3 = adc.read(rate,3)
        
        voltage1 = data_display.format(adc.raw_to_v(sample1))
        voltage2 = data_display.format(adc.raw_to_v(sample2))
        voltage3 = data_display.format(adc.raw_to_v(sample3))
        
        write_oled_menu(text11 = "", 						text12 = "ADC", 				text13 = "",
                        text21 = "CH1", 					text22 = str(voltage1), 		text23 = "",
                        text31 = "CH2", 					text32 = str(voltage2),			text33 = "",
                        text41 = "CH3", 					text42 = str(voltage3), 		text43 = "")
        
    # Change states
    if input == 3: current_state = current_state + 1
    elif input == 1: current_state = current_state - 1
    
    # State overflow
    if current_state > 7 : current_state = 1
    if current_state < 1 : current_state = 7
    
    #Stop annoying buzzer
    if current_state != 5 and current_state != 2 : stateBuzzer = 0
    buzzer.value(stateBuzzer)
        

while(True):
    statemenu(current_state)
    #!Test
    #statemenu(6)

