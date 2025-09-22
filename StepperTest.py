import RPi.GPIO as GPIO # type: ignore
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(40, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)

count = 0
frequencyHz = 2500
prevtime = 0
pulses = 1600*1

try:
    
    print("starting "+str(pulses)+" pulses")
    while count<=pulses*2:
        GPIO.output(38, GPIO.HIGH)
        current_time = time.perf_counter()

        if(current_time-prevtime >= 1/(frequencyHz*2)):
            if(count%2 == 1):
                GPIO.output(40, GPIO.HIGH)

            else:
                GPIO.output(40, GPIO.LOW)

            count = count + 1
            prevtime = current_time

    print("ended")
    GPIO.cleanup()
except:
    print("stopped early")
    GPIO.cleanup()