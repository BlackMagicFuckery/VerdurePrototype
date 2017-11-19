#--------------------------------------
#Verdure Systems Prototype 1.0 Started 2017/09/09
#Hardware - Sparkfun Luminosity Sensor PN: TSL2561
#Hardware - Sparkfun Environmental Combo BME280
#Hardware - Sparkfun Waterproof Temperature Sensor DS18B20
#Hardware - Atlas Scientific pH Sensor Kit XXXXXXX
#Hardware - Atlas Scientific Dissolved Oxygen Sensor XXXXXX
#Hardware - 
#--------------------------------------

### Importing modules

import smbus, time, csv, os, glob
from ctypes import c_short, c_byte, c_ubyte

### A "while" loop to pull continuous data
### n is sample rate in seconds - time.sleep(n) is last line in script
### Note: Sample rate of sensors relevant; stay in spec range, most ~1s max

n=1

while True:
    
####################################################################
#BME280 Air temp,pressure,humidity code, reference sensor data sheet
####################################################################

  DEVICE = 0x77 # Default device i2c address for BME280 sensor

  bus = smbus.SMBus(1) # Rev 2 Pi, Pi 2 & Pi 3 uses bus 1
                       # Rev 1 Pi uses bus 0

  def getShort(data, index):
    # return two bytes from data as a signed 16-bit value
    return c_short((data[index+1] << 8) + data[index]).value

  def getUShort(data, index):
    # return two bytes from data as an unsigned 16-bit value
    return (data[index+1] << 8) + data[index]

  def getChar(data,index):
    # return one byte from data as a signed char
    result = data[index]
    if result > 127:
      result -= 256
    return result

  def getUChar(data,index):
    # return one byte from data as an unsigned char
    result =  data[index] & 0xFF
    return result

  def readBME280ID(addr=DEVICE):
    # Chip ID Register Address
    REG_ID     = 0xD0
    (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
    return (chip_id, chip_version)

  def readBME280All(addr=DEVICE):
        # Register Addresses
    REG_DATA = 0xF7
    REG_CONTROL = 0xF4
    REG_CONFIG  = 0xF5

    REG_CONTROL_HUM = 0xF2
    REG_HUM_MSB = 0xFD
    REG_HUM_LSB = 0xFE

    # Oversample setting - page 27
    OVERSAMPLE_TEMP = 2
    OVERSAMPLE_PRES = 2
    MODE = 1

    # Oversample setting for humidity register - page 26
    OVERSAMPLE_HUM = 2
    bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)

    control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE
    bus.write_byte_data(addr, REG_CONTROL, control)

    # Read blocks of calibration data from EEPROM
    # See Page 22 data sheet
    cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
    cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
    cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)

    # Convert byte data to word values
    dig_T1 = getUShort(cal1, 0)
    dig_T2 = getShort(cal1, 2)
    dig_T3 = getShort(cal1, 4)

    dig_P1 = getUShort(cal1, 6)
    dig_P2 = getShort(cal1, 8)
    dig_P3 = getShort(cal1, 10)
    dig_P4 = getShort(cal1, 12)
    dig_P5 = getShort(cal1, 14)
    dig_P6 = getShort(cal1, 16)
    dig_P7 = getShort(cal1, 18)
    dig_P8 = getShort(cal1, 20)
    dig_P9 = getShort(cal1, 22)

    dig_H1 = getUChar(cal2, 0)
    dig_H2 = getShort(cal3, 0)
    dig_H3 = getUChar(cal3, 2)

    dig_H4 = getChar(cal3, 3)
    dig_H4 = (dig_H4 << 24) >> 20
    dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)

    dig_H5 = getChar(cal3, 5)
    dig_H5 = (dig_H5 << 24) >> 20
    dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)

    dig_H6 = getChar(cal3, 6)

    # Wait in ms (Datasheet Appendix B: Measurement time and current
    # calculation)
    wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM)+0.575)
    time.sleep(wait_time/1000)  # Wait the required time  

    # Read temperature/pressure/humidity
    data = bus.read_i2c_block_data(addr, REG_DATA, 8)
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw = (data[6] << 8) | data[7]

    # Refine temperature
    var1 = ((((temp_raw>>3)-(dig_T1<<1)))*(dig_T2)) >> 11
    var2 = (((((temp_raw>>4) - (dig_T1)) * ((temp_raw>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
    t_fine = var1+var2
    temp_air = float(((t_fine * 5) + 128) >> 8);

    # Refine pressure and adjust for temperature
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * dig_P6 / 32768.0
    var2 = var2 + var1 * dig_P5 * 2.0
    var2 = var2 / 4.0 + dig_P4 * 65536.0
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * dig_P1
    if var1 == 0:
      pressure=0
    else:
      pressure = 1048576.0 - pres_raw
      pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
      var1 = dig_P9 * pressure * pressure / 2147483648.0
      var2 = pressure * dig_P8 / 32768.0
      # Divided next line by 10 to read value in kPa
      pressure = (pressure + (var1 + var2 + dig_P7) / 16.0)/10

    # Refine humidity
    humidity = t_fine - 76800.0
    humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
    humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
    if humidity > 100:
      humidity = 100
    elif humidity < 0:
      humidity = 0

    return temp_air/100.0,pressure/100.0,humidity
  
### Moved this line to an earlier tab for definition of these variables

  temp_air,pressure,humidity = readBME280All()

#######################################################################
# Lux sensor code - TLS2561, found on github
#######################################################################

### Moved this whole block over 1 tab to process at same time as BME280
### sensor readings and uniformity of variable collection times.

  bus.write_byte_data(0x39, 0X00 | 0x80, 0x03)
  bus.write_byte_data(0x39, 0x01 | 0x80, 0x02)
  time.sleep(0.5)
  data = bus.read_i2c_block_data(0x39, 0x0C | 0x80, 2)
  data1 = bus.read_i2c_block_data(0x39, 0x0E | 0x80, 2)
  full_lux = data[1]*256+data[0]
  ir_lux = data1[1]*256+data1[0]
  vis_lux = full_lux-ir_lux

#######################################################################
# Waterproof temp sensor - DS18B20, code from Adafruit
#######################################################################

  os.system('modprobe w1-gpio')
  os.system('modprobe w1-therm')
  base_dir='/sys/bus/w1/devices/'
  device_folder=glob.glob(base_dir+'28*')[0]
  device_file=device_folder+'/w1_slave'

  def read_temp_raw():
      f = open(device_file, 'r')
      lines = f.readlines()
      f.close()
      return lines

  def read_temp():
      lines = read_temp_raw()
      while lines[0].strip()[-3:]!= 'YES':
          time.sleep(0.2)
          lines = read_temp_raw()
      equals_pos = lines[1].find('t=')
      if equals_pos != 1:
          temp_string = lines[1][equals_pos+2:]
          temp_c=float(temp_string)/1000
          return temp_c

    #print round(read_temp(), 1)
  
  #Print sensor values in python shell 

  var_list=time.strftime('%Y%m%d%H%M%S'),ir_lux,vis_lux,full_lux,round(temp_air,1),round(pressure,1),round(humidity,1),round(read_temp(), 1)
  print time.strftime('%Y%m%d%H%M%S')

##########################################################################
#Opening CSV file, writing a header if file is empty and appending data
#as the script loops.
##########################################################################

# Note use of ISO standard for time stamp - YMDHMS format, critical
# for sorting later  

# using csv writer to update all data, use 'ab' for append for binary

  dfwaquadata = [time.strftime("%Y%m%d%H%M%S"),full_lux,ir_lux,vis_lux,round(temp_air,1),round(pressure,1),round(humidity,1),round(read_temp(), 1)]
  todays_file_timestamp = time.strftime('%Y%m%d')
  dfwaquafile = open('DFW_aqua_raw_data_'+todays_file_timestamp+'.csv', 'ab')
  headers = ['Time_stamp YMDHMS', 'Total Lux (Lum/m^2)', 'IR Lux (Lum/m^2)', 'Vis Lux (Lum/m^2)', 'Temp Air (C)', 'Pressure (kPa)', 'Humidity (%)', 'Temp H20 (C)']
  writer = csv.writer(dfwaquafile)
  
  with dfwaquafile as f_output:
    #defining header field and .seek to search the file
    csv_output = csv.DictWriter(f_output, fieldnames = headers)
    f_output.seek(0,2)

    #if the file is empty with .tell "==0" it will write a new header, if not it will only append
    if f_output.tell()==0:
      csv_output.writeheader()

    #write the data to the rows
    writer.writerow(dfwaquadata)

#########################################################################
# Sample rate in seconds - from while loop at top, 'n'
#########################################################################

  time.sleep(n)
