import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

GPIO.setup(40, GPIO.OUT)

count = 0

try:
    print("starting 100000 pulses")
    while count<=100000:

        if(count%1):
            GPIO.output(40, GPIO.HIGH)
        else:
            GPIO.output(40, GPIO.LOW)

        count+=1

    print("ended")
    GPIO.cleanup()
except:
    print("stopped early")
    GPIO.cleanup()