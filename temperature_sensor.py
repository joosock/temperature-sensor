import smbus
import time

PCF8591_ADDRESS = 0x48  
AIN_CHANNEL = 0  

VREF = 3.3  
TEMP_CONVERSION_FACTOR = 100.0  # 10mV/°C

def read_adc(channel):
    
    if channel < 0 or channel > 3:
        raise ValueError("ADC 채널은 0에서 3 사이여야 합니다.")
    
    bus.write_byte(PCF8591_ADDRESS, 0x40 | channel)
    bus.read_byte(PCF8591_ADDRESS)  
    value = bus.read_byte(PCF8591_ADDRESS)
    return value

def convert_to_temperature(adc_value):

    voltage = (adc_value / 255.0) * VREF
    temperature = voltage * TEMP_CONVERSION_FACTOR
    return temperature

if __name__ == "__main__":
    bus = smbus.SMBus(1)  

    try:
        while True:
            adc_value = read_adc(AIN_CHANNEL)
            temperature = convert_to_temperature(adc_value)
            print(f"Temperature: {temperature:.2f} °C")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by User")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bus.close()
