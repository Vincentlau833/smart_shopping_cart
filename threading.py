import RPi.GPIO as GPIO
import time
import threading
import smbus
import math
from hx711 import HX711
import requests

from RPLCD.i2c import CharLCD

api_url = "https://firestore.googleapis.com/v1/projects/smartshoppingcart-4f016/databases/(default)/documents/products"
# firebase configuration
firebaseConfig = {
  "apiKey": "AIzaSyBBU77l4AwzryZVJZFi-Q68ZxYwW495Pys",
  "authDomain": "smartshoppingcart-4f016.firebaseapp.com",
  "databaseURL": "https://smartshoppingcart-4f016-default-rtdb.asia-southeast1.firebasedatabase.app",
  "projectId": "smartshoppingcart-4f016",
  "storageBucket": "smartshoppingcart-4f016.appspot.com",
  "messagingSenderId": "575044201293",
  "appId": "1:575044201293:web:a0db9e36de25d525cd2504"
    
}



GPIO.setmode(GPIO.BCM)

# sensors
# MPU6050 (angle) GPIO 2,3
# HX711 (weight) GPIO 17,18
# Ultrasonic GPIO 22,23
# RFID USB

# Actuators
# Smartphones
# LCD GPIO 2,3
# LED GPIO 26
# Buzzer GPIO 21

# actuators
# LCD
def displayLCD(content,duration,clearDuration):
    lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
    lcd.clear()
    
    lcd.write_string(content)
    time.sleep(duration)
    lcd.clear()
    time.sleep(clearDuration)
    
    
    
# LED
def blink_led(pin):
    
    GPIO.setup(pin, GPIO.OUT)
    
    sleep = 0.06

    for j in range(0, 20):
        for i in range(0, 4):
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(sleep)
            GPIO.output(pin, GPIO.LOW)
            time.sleep(sleep)

        time.sleep(0.6)

    GPIO.cleanup()
    
def lightUpLED(pin,frequency,duration):
    GPIO.setup(pin,GPIO.OUT)
    
    for i in range(0,frequency):
        GPIO.output(pin,GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(pin,GPIO.LOW)
        time.sleep(duration) 

# buzzer
def soundBuzzer(pin,soundFrequency,duration):
    GPIO.setup(pin,GPIO.OUT)
    
    for i in range(0,soundFrequency):
        GPIO.output(pin,GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(pin,GPIO.LOW)
        time.sleep(duration)
        
# sensors
# ultrasonic sensor
def getDistance(trig_pin,echo_pin):
    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    
    # Trigger the sensor by sending a 10us pulse
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    
    # Wait for the echo signal to return
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    # Calculate the distance in centimeters
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance_str = "{:.2f}".format(distance)
    
    time.sleep(0.5)
    return distance
    
# MPU6050(detect angle)
def getAngle():
    bus = smbus.SMBus(1)  # Use bus 1 for Raspberry Pi version 2/3, use 0 for version 1
    MPU6050_ADDRESS = 0x68  # MPU6050 I2C address
    ACCEL_XOUT_H = 0x3B  # Register address for accelerometer data
    
    # Read raw accelerometer data
    data = bus.read_i2c_block_data(MPU6050_ADDRESS, ACCEL_XOUT_H, 6)
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]

    # Convert to signed values
    if accel_x > 32767:
        accel_x -= 65536
    if accel_y > 32767:
        accel_y -= 65536
    if accel_z > 32767:
        accel_z -= 65536

    # Calculate roll and pitch angles in the range 0 to 360 degrees
    roll = (math.atan2(accel_y, accel_z) * 180.0 / math.pi + 360) % 360
    pitch = (math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / math.pi + 360) % 360
    
    # convert the data into 2 decimal string
    roll_str = "{:.2f}".format(roll)
    pitch_str = "{:.2f}".format(pitch)

    return roll, pitch

# hx711(measure weight)
def getWeight(data_pin,sck_pin):
    # Initialize the HX711
    hx = HX711(data_pin, sck_pin)
    
    # Set the measurement parameters (adjust these as needed)
    hx.set_scale_ratio(scale_ratio=-2250)
        
    weight = abs(hx.get_weight_mean()-47)
    weight_str = "{:.2f}".format(weight)
    
    
    return weight
    
# RFID reader
def scan():
    while True:
        productCode = input("Scan : ")
        
        if(productCode == "0008473015"):
            productName = "Pepsi"
            productData = {
            "fields": {
                    "id" : {"stringValue": productCode},
                    "name": {"stringValue": productName},
                    "price": {"integerValue": 10}
                    }
            }
        elif(productCode == "0008892886"):
            productName = "Cola"
            productData = {
                "fields": {
                    "id" : {"stringValue": productCode},
                    "name": {"stringValue": productName},
                    "price": {"integerValue": 10}
                    }
                }
        elif(productCode == "0007684383"):
            productName = "Gardenia"
            productData = {
                "fields": {
                    "id" : {"stringValue": productCode},
                    "name": {"stringValue": productName},
                    "price": {"integerValue": 20}
                    }
                }
        elif(productCode == "0008473004"):
            productName = "Maggie"
            productData = {
                "fields": {
                    "id" : {"stringValue": productCode},
                    "name": {"stringValue": productName},
                    "price": {"integerValue": 30}
                    }
                }
        
        response = requests.post(api_url, json=productData)
        if response.status_code == 200:
            print(productName)
        else:
            print(f"Failed to add document. Status code: {response.status_code}")
        
        displayLCD(productName,2,0)
        
        
    

# cart functions
def detectObjectInfront():
    while True:
        distance = getDistance(22,23)
        #print("Distance: {:.2f} cm".format(distance))
        
        buzzer_on_thread = threading.Thread(target=soundBuzzer,args=(21,1,1,))
        buzzer_off_thread = threading.Thread(target=soundBuzzer,args=(21,0,1,))
        led_on_thread = threading.Thread(target=lightUpLED,args=(26,1,1,))
        led_off_thread = threading.Thread(target=lightUpLED,args=(26,0,1,))
        displayLCD_thread = threading.Thread(target=displayLCD,args=("Too close to the front",1,0,))
        #clearLCD_thread = threading.Thread(target=displayLCD,args=("",))
        
        if(distance < 10):
            buzzer_on_thread.start()
            led_on_thread.start()
            displayLCD_thread.start()
            
            buzzer_on_thread.join()
            led_on_thread.join()
            displayLCD_thread.join()
        else:
            buzzer_off_thread.start()
            led_off_thread.start()
            #clearLCD_thread.start()
            
            buzzer_off_thread.join()
            led_off_thread.join()
            #clearLCD_thread.join()
            
def detectAngle():
    while True:
        buzzer_on_thread = threading.Thread(target=soundBuzzer,args=(21,1,1,))
        buzzer_off_thread = threading.Thread(target=soundBuzzer,args=(21,0,1,))
        led_on_thread = threading.Thread(target=lightUpLED,args=(26,1,1,))
        led_off_thread = threading.Thread(target=lightUpLED,args=(26,0,1,))
        displayLCD_thread = threading.Thread(target=displayLCD,args=("The cart is fall!!!",1,0,))
        #clearLCD_thread = threading.Thread(target=displayLCD,args=("",))
        
        roll, pitch = getAngle()
        time.sleep(1)
#         print(f"Roll: {roll:.2f} degrees")
#         print(f"Pitch: {pitch:.2f} degrees")
        
        if((roll > 50 and roll < 310) or (pitch > 50 and pitch < 310)):
            buzzer_on_thread.start()
            led_on_thread.start()
            displayLCD_thread.start()
            
            buzzer_on_thread.join()
            led_on_thread.join()
            displayLCD_thread.join()
        
        else:
            buzzer_off_thread.start()
            led_off_thread.start()
            #clearLCD_thread.start()
            
            buzzer_off_thread.join()
            led_off_thread.join()
            #clearLCD_thread.join()
            
def detectWeight():
    while True:
        weight = getWeight(17,18)
        
        get_weight_thread = threading.Thread(target=getWeight,args=(17,18,))
        displayLCD_thread = threading.Thread(target=displayLCD,args=("Weight: "+"{:.2f}".format(weight)+" g",8,0))
        
        get_weight_thread.start()
        displayLCD_thread.start()
        
        get_weight_thread.join()
        displayLCD_thread.join()
        #print("Weight: {:.2f} grams".format(weight))
    

    
    
# main function
def main():
    try:
        detect_angle_thread = threading.Thread(target=detectAngle)
        detect_object_thread = threading.Thread(target=detectObjectInfront)
        detect_weight_thread = threading.Thread(target=detectWeight)
        detect_product_thread = threading.Thread(target=scan)
        
        
        detect_angle_thread.start()
        detect_object_thread.start()
        detect_weight_thread.start()
        detect_product_thread.start()
        
        detect_angle_thread.join()
        detect_object_thread.join()
        detect_weight_thread.join()
        detect_product_thread.join()
        
    except KeyboardInterrupt:
        pass

# if __name__ == "__main__":
#     main()

scan()

# while True:
#     print("Weight : "+getWeight(17,18)+" grams")
   
#     print("Distance : "+getDistance(22,23)+" cm")
#     time.sleep(1)


# while True:
#      detectObjectInfront()
#      detectAngle()
#      detectWeight()



