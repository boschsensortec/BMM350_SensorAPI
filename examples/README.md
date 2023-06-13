# BMM350 Examples

Commands to run tests on MCU:

To run in Command line :
---------------------------
1. mingw32-make clean
2. mingw32-make TARGET=MCU_APP30 all
3. mingw32-make TARGET=MCU_APP30 download

Commands to run tests on PC:

To run in Command line :
---------------------------
1. mingw32-make clean
2. mingw32-make all

### Example 1 : bmm3530 config changes:

    This example is to test the change in configurations like powermode, ODR and AVG, axis enabling/disabling etc.

#### Procedure:

1. Check chip_id
2. Set normal mode
3. Read data and print
4. Calculate mean and noise for 100 samples
5. Reconfigure ODR and AVG( 50,8x / 400,1x / 25,4x ) in normal mode
6. Set suspend mode
7. Read data and print (10x loop) -> no change 
8. Disable x and y axis alone
9. Set normal mode
10. Read 20x and print 

#### Usecase:

1. To test the behaviour of sensor in normal mode and suspend mode   

### Example 2 : bmm350 forced mode:

    This example is to read the mag data in forced mode under various configurations
    
#### Procedure:

For combination 1,

    Set forced mode fast and read data with averaging between 4 samples
    
    Usecase: No change is data so the customer understands that forced mode just does 1 conversion.

For combination 2,

    Set forced mode fast and read data with averaging between 4 samples in a loop
    
    Usecase: The value changes for every read 
    
For combination 3,

    Set forced mode and read data with no averaging between samples in a loop
         
	Usecase: Customer understands how AVG setting affects forced mode
    
For combination 4,

    Set forced mode fast and read data with averaging between 4 samples in a loop

    Usecase: The value changes for every read 
    
For combination 5,

    Set forced mode and read data with no averaging between samples in a loop
    
	Usecase: Customer understands how AVG setting affects forced mode

For combination 6,

    Set forced mode fast and read data with averaging between 2 samples in a loop

    Usecase: The value changes for every read. Also customer understands that a small AVG setting requires less time for a conversion.

### Example 3 : bmm350 illegal command:
    
    This example is to read the flag bit PMU_CMD_STATUS_0.cmd_is_illegal. When PMU_CMD register is set to illegal values, then the flag bit is set.
    
#### Procedure:

1. Read chip id
2. Set legal PMU_CMD (0 to 8)
3. Read PMU_CMD_STATUS_0.cmd_is_illegal
4. Set illegal PMU_CMD (10 to 15)
5. Read PMU_CMD_STATUS_0.cmd_is_illegal  

#### Usecase:

1. Customer understands bmm350 is robust against bad commands sent to PMU_CMD

### Example 4 : bmm350 interrupt:

    This example is to test the interrupts by changing the interrupt pin configurations. 
    
#### Procedure:

1. Check Chip id
2. Configure interrupt( active HIGH, pulsed, mapped to INT pin, push-pull)
3. Wait for 10 seconds
4. Change different ODR
5. Wait for 10 seconds
6. Configure interrupt( active LOW, pulsed, mapped to INT pin, push-pull)
7. Wait for 10 seconds
8. Configure interrupt( active LOW, latched, mapped to INT pin, push-pull)
9. Wait for 10 seconds
10. Configure interrupt( active HIGH, pulsed, mapped to INT pin, push-pull)

#### Usecase:

1. Customer should use oscilloscope to watch INT pin pulsing
2. Customer will see INT going LOW (active LOW) and stay there

##### Scenario 1:

    Read magnetic and temperature data with 100ms delay for 20 times
 
##### Usecase: 

    Customer will understand that clearing interrupt status requires reading data

##### Scenario 2:

    Read magnetic and temperature data with hardware interrupt by reading INT_STATUS register for 20 times

##### Usecase:

    Customer will understand that clearing interrupt status can also be done by reading INT_STATUS

### Example 5 : bmm350 magnetic reset:

    This example is to understand what to do to trigger magnetic reset , e.g. after magnetic shock or longer idle times in suspend
    
#### Procedure:

1. Set normal mode
2. Read magnetic and temperature data for 20 times and print
3. Set suspend mode
4. Send magnetic reset
5. Set normal mode
6. Read magnetic and temperature data for 20 times and print
7. Send magnetic reset
8. Read magnetic and temperature data for 20 times and print

#### Usecase:

1. Customer understands that API function checks for power mode. It automatically switches to suspend  and restores normal mode if this was the mode selected

### Example 6 : bmm350 polling:

    This example is to read the magnetic and temperature data by polling machanism.
    
#### Procedure:

##### Scenario 1:

    1. Set normal mode
    2. Set ODR = 25Hz, AVG = 8x
    3. Loop = 20
    4. Wait for 36ms (1 ODR period - 10% margin)
    5. Read the magnetic and temperature data and print

##### Usecase:

    Customer understands that too fast reading leads to double reads. too slow reading thus leads to loss of data.
    
##### Scenario 2:

    1. Set normal mode
    2. Set ODR = 25Hz, AVG = 8x
    3. Loop = 22
    4. Read the magnetic and temperature data by reading INT_STATUS register and print 

##### Usecase:

    Customer understands data loss can be prevented by reading data ready flag before reading data.
    
    
### Example 7 : bmm350 self test:

    This example is to test the selftest on various power modes
    
#### Procedure:

##### Scenario 1:

1. Set normal mode
2. Loop = 10
3. Read magnetometer and temperature data 
4. Provide 100ms delay
5. Print magnetometer and temperature data 

##### Scenario 2:

1. Set suspend mode
2. Loop = 20
3. Call bmm350_ perform_ self_test API
4. Print selftest data
5. Loop 20 times and print magnetic and temperature data

##### Usecase:

    Customer understands selftest is called from suspend mode

### Example 8 : bmm350 sensor time:

    This example is to test sensortime under various power mode configurations.
    
#### Procedure:

##### Scenario 1:

    1. Set suspend mode
    2. Read sensortime and print for 20 loop

##### Usecase:

    Customer understands : no change in suspend mode.
    
##### Scenario 2:

    1. Set CTRL_USER to 0x01 
    2. Loop = 20
    3. Set forced mode
    4. Wait for 40ms
    5. Read sensortime (Change in data)
    6. Read sensortime (No change in data)
    7. Read sensortime (No change in data)

##### Usecase:

    Customer understands: sensortime is only updated when a conversion happens.
sensortime indicates the SENSOR time at which  the last conversion happened

##### Scenario 3:

    1. Set normal mode
    2. Set ODR = 100hz, AVG = 2x
    3. Read sensortime and print for 20 loop (change in data everytime)

##### Usecase:

    Customer understands : at every conversion sensortime is updated
this program requires compilation on the ARM since the wait time is short
            
### Example 9 : bmm350 normal mode:

    This example is to read the uncompensated and compensated magnetic and temperature data in normal mode.

#### Procedure:

1. Read chip id
2. Configure interrupt(active HIGH, pulsed, unmap from pin, push-pull) 
3. Enable data ready interrupt
4. Set ODR and AVG
5. Enable all axes
6. Set normal mode
7. Read uncompensated magnetometer and temperature data for 20 times by reading INT_STATUS register 
8. Read compensated magnetometer and temperature data for 50 times by reading INT_STATUS register 

#### Usecase:

    To find the difference between uncompensated and compensated magnetic and temperature data read in normal mode.

### Example 10 : bmm350 out of range detect:

    This example is to measure the data from the sensor and detects when sensor goes out of range i.e greater than +/- 2400uT
with half self-test threshold and full self-test threshold

#### Procedure:

1. Read chip id
2. Perform self-test
3. Set ODR and AVG
4. Enable all axes
5. Set forced mode fast
6. Measure data such as 
	a. Magnetometer x, y and z axis
	b. Field strength
	c. Out of range
	d. X failed and Y failed
	e. Difference in magnetometer positive x and negative x and difference in magnetometer positive y and negative y
	f. Reset value and Reset counter
	g. Self-test active
	h. Self-test counter
7. Trigger magnetic reset

#### Usecase:

    To find the sensor state when magnetometer value goes out of range detection
