import board
import countio
import time
import supervisor
import array
import simpleio
import asyncio
import buttons
import display
import displayio
import system_data
from adafruit_display_text import label
import throttle
import brake
import wheel_speed_sensor
import torque_sensor
import motor_temperature_sensor
import vesc
import display
import esp32
import terminalio
import time
import gc
import pas_sensor
from microcontroller import watchdog
from watchdog import WatchDogMode

import supervisor
supervisor.runtime.autoreload = False

# Tested on a QtPy32-S3

###############################################
# OPTIONS

torque_sensor_weight_min_to_start_x10 = 40 # (value in kgs) let's avoid any false startup, we will need this minimum weight on the pedals to start
torque_sensor_weight_max_x10 = 400 # torque sensor max value is 40 kgs. Let's use the max range up to 40 kgs

motor_min_current_start = 1.0 # to much lower value will make the motor vibrate and not run, so, impose a min limit (??)
motor_max_current_limit = 7.0 # max value, be carefull to not burn your motor

ramp_up_time = 0.1 # ram up time for each 1A
ramp_down_time = 0.05 # ram down time for each 1A

throttle_enable = True # should throttle be used?

cranck_lenght_mm = 170

pas_prev_count =0

throttle_adc_min = 14650 # this is a value that should be a bit superior than the min value, so if throttle is in rest position, motor will not run
throttle_adc_max = 61940
throttle_adc_over_max_error = 61960 # this is a value that should be a bit superior than the max value, just to protect is the case there is some issue with the signal and then motor can keep run at max speed!!

# debug options
enable_print_ebike_data_to_terminal = True
enable_debug_log_cvs = False

# ramp up and down constants
current_ramp_up_time = 0.00001 # ram up time for each 1A
current_ramp_down_time = 0.00001 # ram down time for each 1A
speed_ramp_up_time = 0.00001 # ram up time for each 1 erpm
speed_ramp_down_time = 0.00001 # ram down time for each 1 erpm

# Lunyee fast motor 12 inches (not the original Fiido Q1S motor) has 15 poles pair
motor_poles_pair = 15

# original Fiido Q1S 12 inches wheels are 305mm in diameter
wheel_circunference = 305.0

# max wheel speed in ERPM
motor_max_speed_limit = 13100 # 50kms/h

throttle_lowpass_filter_state = None

class MotorControlScheme:
    CURRENT = 0
    SPEED_CURRENT = 1


class InputControlMode: 
    PAS_MODE = 0
    THROTTLE_MODE = 1

################################################
#  DISPLAY
################################################

displayObject = display.Display()

display = displayObject.display

DISPLAY_HEIGHT = 128
DISPLAY_WIDTH = 64  
TEXT = "0"

display.rotation = 180

# screen 1
label_x = 10
label_y = 18
label_1 = label.Label(terminalio.FONT, text=TEXT)
label_1.anchor_point = (0.0, 0.0)
label_1.anchored_position = (label_x, label_y)
label_1.scale = 1
label_1.text = "Ready to power on"

screen1_group = displayio.Group()
screen1_group.append(label_1)
display.show(screen1_group)

#time_previous = time.monotonic()


assist_level_area = label.Label(terminalio.FONT, text=TEXT)
assist_level_area.anchor_point = (0.0, 0.0)
assist_level_area.anchored_position = (4, 0)
assist_level_area.scale = 2

battery_voltage_area = label.Label(terminalio.FONT, text=TEXT)
battery_voltage_area.anchor_point = (1.0, 0.0)
battery_voltage_area.anchored_position = (128, 0)
battery_voltage_area.scale = 1

label_x = 61
label_y = 10 + 16
label_1 = label.Label(terminalio.FONT, text=TEXT)
label_1.anchor_point = (1.0, 0.0)
label_1.anchored_position = (label_x, label_y)
label_1.scale = 3

label_x = 129
label_3 = label.Label(terminalio.FONT, text=TEXT)
label_3.anchor_point = (1.0, 0.0)
label_3.anchored_position = (label_x, label_y)
label_3.scale = 3

warning_area = label.Label(terminalio.FONT, text=TEXT)
warning_area.anchor_point = (0.0, 0.0)
warning_area.anchored_position = (2, 48)
warning_area.scale = 1

text_group = displayio.Group()
# text_group.append(assist_level_area)
text_group.append(battery_voltage_area)
text_group.append(label_1)
# text_group.append(label_2)
text_group.append(label_3)
text_group.append(warning_area)

display.show(text_group)



################################################
motor_control_scheme = MotorControlScheme.CURRENT
# motor_control_scheme = MotorControlScheme.SPEED_CURRENT

input_control_mode = InputControlMode.THROTTLE_MODE

###############################################

assist_level_factor_table = [
    0,
    0.13,
    0.16,
    0.20,
    0.24,
    0.31,
    0.38,
    0.48,
    0.60,
    0.75,
    0.93,
    1.16,
    1.46,
    1.82,
    2.27,
    2.84,
    3.55,
    4.44,
    5.55,
    6.94,
    8.67
]

# open file for log data
if enable_debug_log_cvs:
    log = open("/log_csv.txt", "w")

pas_sensor = countio.Counter(board.D8,)

brake = brake.Brake(
   board.D18) # A0 brake sensor pin

wheel_speed_sensor = wheel_speed_sensor.WheelSpeedSensor(
   board.D17) # wheel speed sensor pin
   
torque_sensor = torque_sensor.TorqueSensor(
    board.D35, # CAN tx pin
    board.D37) # CAN rx pin
  
throttle = throttle.Throttle(
    board.A2, # ADC pin for throttle
    throttle_adc_min, # min ADC value that throttle reads, plus some margin
    throttle_adc_max) # max ADC value that throttle reads, minus some margin

#motor_temperature_sensor = motor_temperature_sensor.MotorTemperatureSensor(
 #  board.D8) # motor temperature sensor pin

esp32 = esp32.ESP32()

system_data = system_data.SystemData()

vesc = vesc.Vesc(
    board.TX, # UART TX pin tebike_app_datahat connect to VESC
    board.RX, # UART RX pin that connect to VESC
    system_data) #VESC data object to hold the VESC data



def check_brakes():
    """Check the brakes and if they are active, set the motor current to 0
    """
    if system_data.brakes_are_active == False and brake.value == True:
        print("bk" ,brake.value)
        # brake / coast the motor
        vesc.brake()
        system_data.motor_target_current = 0
        system_data.brakes_are_active = True
      
    elif system_data.brakes_are_active == True and brake.value == False:
        print("bk OFF")
        system_data.brakes_are_active = False
        
def lowpass_filter(sample, filter_constant):
    global throttle_lowpass_filter_state

    # initialization
    if throttle_lowpass_filter_state is None:
        throttle_lowpass_filter_state = sample

    throttle_lowpass_filter_state = throttle_lowpass_filter_state - ((filter_constant) * ((throttle_lowpass_filter_state) - (sample)))

    return throttle_lowpass_filter_state

array_meanA = array.array('I', (0 for _ in range(10)))

def pas_check_count():
    global pas_prev_count
    global array_meanA
    n = 10
    count = pas_sensor.count
    if  count > pas_prev_count:
        diff = count - pas_prev_count
        array_meanA.append(diff)                    # count diff slots into end of array.
        for h in range(n):
            array_meanA[h] = array_meanA[(h+1)]     # Shift the values in the array to the left
        array_meanA.pop()                           # remove the last entry
        meanA = 0
        for h in range(n):
            meanA = array_meanA[h] + meanA          # Calculate the mean, no weights.
        meanA = meanA/n
    pas_prev_count = count
    if meanA > 6:
        return True
    else:
        return False

def filter_motor_power(motor_power):
    
    if motor_power < 0:
        if motor_power > -10:
            motor_power = 0
        elif motor_power > -25:
            pass
        elif motor_power > -50:
            motor_power = round(motor_power / 2) * 2 
        elif motor_power > -100:
            motor_power = round(motor_power / 5) * 5
        else:
            motor_power = round(motor_power / 10) * 10        
    else:
        if motor_power < 10:
            motor_power = 0
        elif motor_power < 25:
            pass
        elif motor_power < 50:
            motor_power = round(motor_power / 2) * 2 
        elif motor_power < 100:
            motor_power = round(motor_power / 5) * 5
        else:
            motor_power = round(motor_power / 10) * 10

    return motor_power

def print_ebike_data_to_terminal():
    
    check_brakes()
    
    """Print EBike data to terminal
    """
    if system_data.battery_current_x100 < 0:
       system_data.battery_current_x100 = 0

    if system_data.motor_current_x100 < 0:
       system_data.motor_current_x100 = 0
  
   #print(f" {system_data.motor_target_current:2.1f} | {system_data.motor_current_x100:2.1f} | {system_data.battery_current_x100:2.1f}", end='\r')
    #print(f" {system_data.torque_weight: 2.1f} | {system_data.cadence: 3}", end='\r')
    #print(f"{throttle.adc_value:6} | {(throttle.value / 10.0):2.1f} %", end='\r')
    #print(f" {system_data.motor_current_x100:2.1f} | {system_data.battery_current_x100:2.1f} | {system_data.battery_voltage_x10:2.1f} | {int(system_data.motor_power)}")
    #print(f"{(esp32.temperature_x10 / 10.0):3.1f} | {(system_data.vesc_temperature_x10 / 10.0):3.1f} | {(motor_temperature_sensor.value_x10  / 10.0):3.1f}")
    print(f"mtarget {system_data.motor_target_current:3} | ADC {throttle.adc_value:6} | THROTTLE POS {(throttle.value / 10.0):2.1f} % | BRAKE {brake.value:1} | CPU TEMP {(esp32.temperature_x10 / 10.0):3.1f} | MOTOR REQUESTED CURRENT {system_data.motor_target_current:2.1f} | {system_data.motor_target_current:2.1f} | {system_data.motor_current_x100:2.1f} | {system_data.battery_current_x100:2.1f}", end='\r\n')

def utils_step_towards(current_value, target_value, step):
    """ Move current_value towards the target_value, by increasing / decreasing by step
    """
    value = current_value

    if current_value < target_value:
        if (current_value + step) < target_value:
            value += step
        else:
            value = target_value
    
    elif current_value > target_value:
        if (current_value - step) > target_value:
            value -= step
        else:
            value = target_value

    return value

async def task_log_data():
    while True:
        # log data to local file system CSV file
        brake = 0
        if brake.value:
            brake = 1
        log.write(f"{system_data.torque_weight_x10:.1f},{system_data.cadence},{brake},{system_data.battery_current_x100:.1f},{system_data.motor_current_x100:.1f}\n")
        print(supervisor.ticks_ms())

        system_data.log_flush_cnt += 1
        if system_data.log_flush_cnt > 200:
            system_data.log_flush_cnt = 0
            log.flush()

        # idle 25ms, fine tunned
        await asyncio.sleep(0.025)

async def task_display_process_data():
    while True:
        check_brakes()      # are breaks active and we should disable the motor?

        now = time.monotonic()
        
        assist_level = 0
        assist_level_state = 0
        assist_level_time_previous = now
        display_time_previous = now
        ebike_rx_tx_data_time_previous = now
        power_switch_send_data_time_previous = now

        battery_voltage_previous_x10 = 9999
        battery_current_previous_x100 = 9999
        motor_current_previous_x100 = 9999
        motor_power_previous = 9999
        motor_temperature_sensor_x10_previous = 9999
        vesc_temperature_x10_previous = 9999
        motor_speed_erpm_previous = 9999
        brakes_are_active_previous = False
        vesc_fault_code_previous = 9999
        
        if battery_voltage_previous_x10 != system_data.battery_voltage_x10:
            battery_voltage_previous_x10 = system_data.battery_voltage_x10
            battery_voltage = system_data.battery_voltage_x10 / 10.0
            battery_voltage_area.text = f"{battery_voltage:2.1f}v"
        
        system_data.motor_power = int((system_data.battery_voltage_x10 * system_data.battery_current_x100) / 1000.0)
        if motor_power_previous != system_data.motor_power:
            motor_power_previous = system_data.motor_power
            motor_power = filter_motor_power(system_data.motor_power)
            label_1.text = '{:3}'.format(int(throttle.value / 10))
        
        if motor_speed_erpm_previous != system_data.motor_speed_erpm:
            motor_speed_erpm_previous = system_data.motor_speed_erpm

            # Fiido Q1S original motor runs 45 ERPM for each 1 RPM
            # calculate the wheel speed
            wheel_circunference = 305.0
            perimeter = 6.28 * wheel_circunference
            # motor_rpm = system_data.motor_speed_erpm / 45.0
            motor_rpm = system_data.motor_speed_erpm / 15.0
            speed = ((perimeter / 1000.0) * motor_rpm * 60)
            if speed < 0.1:
                speed = 0.0

            label_3.text = f"{speed:2.1f}"
            
        now = time.monotonic()
            
        #if (now - power_switch_send_data_time_previous) > 0.25:
            #power_switch_send_data_time_previous = now

            #system_data.display_communication_counter = (system_data.display_communication_counter + 1) % 1024
            #power_switch.update()

        if brakes_are_active_previous != system_data.brakes_are_active:
            brakes_are_active_previous = system_data.brakes_are_active
            if system_data.brakes_are_active:
                warning_area.text = str("brakes")
            else:
                warning_area.text = str("")
        elif vesc_fault_code_previous != system_data.vesc_fault_code:
            vesc_fault_code_previous = system_data.vesc_fault_code
            if system_data.vesc_fault_code:
                warning_area.text = str(f"mot e: {system_data.vesc_fault_code}")
            else:
                warning_area.text = str("")
                
        await asyncio.sleep(0.2)
        
async def task_vesc_heartbeat():
    while True:
        # are breaks active and we should disable the motor?
        check_brakes()
        
        # VESC heart beat must be sent more frequently than 1 second, otherwise the motor will stop
        vesc.send_heart_beat()
        
        # ask for VESC latest data
        vesc.refresh_data()

        # let's calculate here this:
        system_data.motor_power = system_data.battery_voltage_x10 * system_data.battery_current_x100

        # should we print system_data data to terminal?
        if enable_print_ebike_data_to_terminal == True:
            print_ebike_data_to_terminal()

        # idle 500ms
        await asyncio.sleep(0.5)

motor_target_current__torque_sensor = 0

async def motor_control():
    while True:
        #check_brakes()
        ##########################################################################################
        # Throttle
        # map torque value.
        motor_max_current_target = motor_max_current_limit
        motor_max_speed_target = motor_max_speed_limit

                          
        # low pass filter throttle to smooth it,
        # because the easy DIY hardware lacks such low pass filter on purpose
        throttle_value_filtered = lowpass_filter(throttle.value, 0.25)
        if throttle_value_filtered < 1.0:
            throttle_value_filtered = 0
        #throttle_value_filtered = throttle.value
        #print("tval", throttle.value)

            # check to see if throttle is over the suposed max error value,
            # if this happens, that probably means there is an issue with ADC and this can be dangerous,
            # as this did happen a few times during development and motor keeps running at max target / current / speed!!
            # the raise Exception() will reset the system
        throttle_adc_previous_value = throttle.adc_previous_value
        if throttle_adc_previous_value > throttle_adc_over_max_error:
            # send 3x times the motor current 0, to make sure VESC receives it
            vesc.set_motor_current_amps(0)
            vesc.set_motor_current_amps(0)
            vesc.set_motor_current_amps(0)
            print(f"throttle_adc_previous_value: {throttle_adc_previous_value}")
            raise Exception("throttle value is over max, this can be dangerous!")

        motor_target_current = simpleio.map_range(
            throttle_value_filtered,
            0.0, # min input
            1000.0, # max input
            motor_min_current_start, # min output
            motor_max_current_target) # max output

        motor_target_speed = simpleio.map_range(
            throttle_value_filtered,
            0.0, # min input
            1000.0, # max input
            950.0, # min output
            motor_max_speed_target) # max output
        ##########################################################################################
        #print("mtarget", motor_target_current)
                    

        # impose a min motor target value, as to much lower value will make the motor vibrate and not run (??)
        if motor_target_current < (motor_min_current_start):
            motor_target_current = 0.0

        if motor_target_speed < 1200.0:
            motor_target_speed = 0.0

        # apply ramp up / down factor: faster when ramp down
        if motor_target_current > system_data.motor_target_current:
            ramp_time_current = current_ramp_up_time
        else:
            ramp_time_current = current_ramp_down_time

        if motor_target_speed > system_data.motor_target_speed:
            ramp_time_speed = speed_ramp_up_time
        else:
            ramp_time_speed = speed_ramp_down_time

        time_now = time.monotonic_ns()
        ramp_step = (time_now - system_data.ramp_current_last_time) / (ramp_time_current * 40_000_000)
        system_data.ramp_current_last_time = time_now
        system_data.motor_target_current = utils_step_towards(system_data.motor_target_current, motor_target_current, ramp_step)

        time_now = time.monotonic_ns()
        ramp_step = (time_now - system_data.ramp_speed_last_time) / (ramp_time_speed * 40_000_000)
        system_data.ramp_speed_last_time = time_now
        system_data.motor_target_speed = utils_step_towards(system_data.motor_target_speed, motor_target_speed, ramp_step)

        # let's limit the value
        if system_data.motor_target_current > motor_max_current_limit:
            system_data.motor_target_current = motor_max_current_limit

        if system_data.motor_target_speed > motor_max_speed_limit:
            system_data.motor_target_speed = motor_max_speed_limit

        # limit very small and negative values
        if system_data.motor_target_current <= 1.0:
            system_data.motor_target_current = 0.0

        if system_data.motor_target_speed < 949.0: # 1kms/h
            system_data.motor_target_speed = 0.0

        # if brakes are active, set our motor target as zero
        system_data.brakes_are_active = False
        if brake.value:
            system_data.motor_target_current = 0.0
            system_data.motor_target_speed = 0.0
            system_data.brakes_are_active = True

        # if motor state is off, set our motor target as zero
        #if system_data.motor_enable_state == False:
           # system_data.motor_target_current = 0.0
            #system_data.motor_target_speed = 0.0

        if motor_control_scheme == MotorControlScheme.CURRENT:
            vesc.set_motor_current_amps(system_data.motor_target_current)
        elif motor_control_scheme == MotorControlScheme.SPEED_CURRENT:
            vesc.set_motor_speed_erpm(system_data.motor_target_speed)

        # we just updated the motor target, so let's feed the watchdog to avoid a system reset
        #watchdog.feed() # avoid system reset because watchdog timeout

        gc.collect() # https://learn.adafruit.com/Memory-saving-tips-for-CircuitPython

        # idle 20ms
        await asyncio.sleep(0.02)

async def main():

    print("starting")

    vesc_heartbeat_task = asyncio.create_task(task_vesc_heartbeat())
    read_sensors_control_motor_task = asyncio.create_task(motor_control())
    display_process_data_task = asyncio.create_task(task_display_process_data())

    # Start the tasks. Note that log_data_task may be disabled as a configuration
    if enable_debug_log_cvs == False:
        await asyncio.gather(
            vesc_heartbeat_task,
            read_sensors_control_motor_task,
            display_process_data_task)
    else:
        log_data_task = asyncio.create_task(task_log_data())
        await asyncio.gather(
            vesc_heartbeat_task,
            read_sensors_control_motor_task,
            display_process_data_task,
            log_data_task)
  
    print("done main()")

asyncio.run(main())