#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// Defines 

#define MCP4725_CMD_WRITEDAC 0x40 // Command to write to the DAC register
#define MCP4725_CMD_WRITEDACEEPROM 0x60 // Command to write to the EEPROM register

bool MCP4725_setVoltage(i2c_inst_t * i2c_port, uint8_t i2c_addr, bool writeEEprom, uint16_t value, uint32_t speed) {

    i2c_set_baudrate(i2c_port, speed); // Set the I2C speed

    uint8_t buffer[3];

    if(writeEEprom) {
        buffer[0] = MCP4725_CMD_WRITEDACEEPROM; // Command to write to EEPROM
    } else {
        buffer[0] = MCP4725_CMD_WRITEDAC; // Command to write to DAC register
    }

    buffer[1] = value / 16; // upper 4 bits of the 12 bit value
    buffer[2] = (value %16) << 4; // Shift the last 4 bits to the left
      // Attempt to write all 3 bytes
    int written = i2c_write_blocking(i2c_port, i2c_addr, buffer, 3, false);
    return (written == 3);

}


//I2c Defines
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define DAC_ADDRESS 0x62 //Default address no jumpers. 


#define THRESHOLD_LO 2000
#define THRESHOLD_HI 2100




float measure_frequency_live_adc() {
    // Wait for the signal to go below THRESHOLD_LO
    while (adc_read() > THRESHOLD_LO) {
        tight_loop_contents(); 
    }

    // Once below, wait for it to go above THRESHOLD_HI
    while (adc_read() < THRESHOLD_HI) {
        tight_loop_contents(); 
    }

    // First crossing -> start timer
    uint64_t t_start = time_us_64();

    // Wait again for below -> above sequence
    while (adc_read() > THRESHOLD_LO) {tight_loop_contents();}
    while (adc_read() < THRESHOLD_HI) {tight_loop_contents();}

    // Second crossing -> stop timer
    uint64_t t_end = time_us_64();

    // Period in microseconds
    uint64_t period_us = t_end - t_start;
    if (period_us == 0) return 0;
    // printf("Period: %llu us\n", period_us);
    // Convert to frequency
    float freq = 1e6f / (float)period_us;
    return freq;
}

float average_freq(float freq, uint16_t count){
    uint32_t sum = 0; 
    for(int i = 0; i < count; i++){
        //printf("Frequency %d: %f\n", i, freq);
        sum += freq; 
    }
    return sum/count;
}


#define AVG_COUNT 32
#define ERROR_TOLERANCE_HZ 1.0f
#define MAX_DAC 4095
#define DAC_STEP 10 // Small step for ramp tuning

#define up_button 2
#define down_button 3


static float freq_buffer[AVG_COUNT];
uint16_t calibrated_dac_values[8]; // Final tuned DAC values per note



int main()
{
    stdio_init_all();

    adc_init(); // Initialise the ADC
    adc_gpio_init(26); // Initialise GPIO26 for ADC input
    adc_select_input(0); // Select ADC input 0 (GPIO26)

    // I2C Initialisation. Using it at 400kHz.
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    printf("I2C Initialised\n");


    gpio_init(up_button);
    gpio_init(down_button);
    gpio_set_dir(up_button, GPIO_IN);
    gpio_set_dir(down_button, GPIO_IN);
    gpio_pull_up(up_button);
    gpio_pull_up(down_button);

    uint16_t dac_Oct_values[] = {
        0,    // 0V
        493,  // ~0.397V
        987,  // ~0.794V
        1481, // ~1.190V
        1975, // ~1.587V
        2469, // ~1.984V
        2963, // ~2.381V
        3457, // ~2.778V
        4095  // 3.3V (full scale)
    };
    
    
    float note_C_frequencies[] = {
        16.35,   // C0
        32.70,   // C1
        65.41,   // C2
        130.81,  // C3
        261.63,  // C4 (Middle C)
        523.25,  // C5
        1046.50, // C6
        2093.00, // C7
        4186.01  // C8
    };
    
    float freq_buffer[AVG_COUNT];
    float freq = 0; 


    int count = 0;

    bool pressed = false;
    bool ok;
    
    while (true) {


        
       pressed = false;
        printf("waiting for button input\n");
        while(!pressed){
            if(gpio_get(up_button) == 0){
                printf("up button pressed\n");
                sleep_ms(10);
                count++;
                if(count % 10 ==0){
                    count = 0;
                }
                printf("Count: %d\n", count);
                pressed = true;
            } else if(gpio_get(down_button) == 0){
                printf("down button pressed\n");
                sleep_ms(10);
                count--;
                if(count < 0){
                    count = 0;
                }
                printf("Count: %d\n", count);
                pressed = true;
            } else {
                pressed = false;
            }
        }
        sleep_ms(200);

        ok = MCP4725_setVoltage(I2C_PORT, DAC_ADDRESS, false, dac_Oct_values[count], 1000 * 1000);
            if (!ok) {
                printf("Failed to set voltage\n");
            } else {
                printf("Voltage set to DAC value: %d and the frequency being referenced is %d\n", dac_Oct_values[count], (uint32_t)note_C_frequencies[count]);
            }
        float freqAvg;
        freq = measure_frequency_live_adc();
        freqAvg = average_freq(freq, AVG_COUNT);
        printf("Average Frequency: %f Hz\n", freqAvg);
        freqAvg = 0;
        


        
       

        /*
        for (int i = 0; i < 9; i++) {
            bool ok = MCP4725_setVoltage(I2C_PORT, DAC_ADDRESS, false, dac_Oct_values[i], 1000 * 1000);
            if (!ok) {
                printf("Failed to set voltage\n");
            } else {
                printf("Voltage set to DAC value: %d and the frequency being referenced is %d\n", dac_Oct_values[i], (uint32_t)note_C_frequencies[i]);
            }

            
            
            
            float freqAvg;
            freq = measure_frequency_live_adc();
            freqAvg = average_freq(freq, AVG_COUNT);
            printf("Average Frequency: %f Hz\n", freqAvg);
            freqAvg = 0;
            sleep_ms(2000);
        }
        */
        
        
        
    }

    return 0;
}
