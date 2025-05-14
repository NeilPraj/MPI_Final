#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/mutex.h"

#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define CV_ADDRESS 0x62
#define PWM_ADDRESS 0x63

#define Button1 6
#define Button2 7
#define Button3 8
#define Button4 9
#define LED1 10
#define LED2 11
#define LED3 12

#define MCP4725_CMD_WRITEDAC 0x40       // Command to write to the DAC register
#define MCP4725_CMD_WRITEDACEEPROM 0x60 // Command to write to the EEPROM register

#define THRESHOLD 2048 //Threshold to detect new period
#define MIN_PERIOD_US 225 // 225 uis = 4444 hz. Any period larger than this is ignored to prevent CPU overuse. 
#define NUM_PERIODS 4 //Number of periods to average over

#define NUM_TUNE_POINTS 9 //Tune over 8 octaves



mutex_t freq_mutex;
mutex_t button_mutex;
mutex_t note_mutex;
mutex_t note_selected_mutex;

uint32_t calibration_table[4096] = {0};

const static double initial_note_frequency = 16.35; // C0 frequency. Change to whatever your lowest reference note is

struct noteData
{
    double expected_frequency;
    double actual_frequency;
    double expected_dac;
    uint32_t actual_dac;
};

struct noteData test_points[NUM_TUNE_POINTS] = {};

struct tunable
{
    struct noteData *noteData;
    int num_tune_points;
    uint32_t *dac_calibration_table;
    uint32_t dac_size;
};

struct tunable tuned_notes = {
    .noteData = test_points,
    .num_tune_points = NUM_TUNE_POINTS,
    .dac_calibration_table = calibration_table,
    .dac_size = 4096 // 12 bit DAC
};

struct note
{
    char letter;
    int octave;
};

volatile struct note current_note = {'A', 4};

// Note letters and numbers
const char note_letters[] = {'C', 'D', 'E', 'F', 'G', 'A', 'B'};
const int NUM_NOTE_LETTERS = sizeof(note_letters) / sizeof(note_letters[0]);

volatile int note_letter_index = 0;
volatile int note_octave = 4;

// ADC values
volatile uint32_t prev_adc_value = 0;
volatile uint64_t last_edge_time = 0;
volatile float measured_frequency = 0.0f;
volatile uint64_t periods[NUM_PERIODS] = {0}; // Store last 4 periods
volatile int period_index = 0;
volatile int periods_collected = 0;
volatile uint64_t period_sum = 0; // Running sum of periods
volatile bool freq_valid = false;

volatile bool tune_flag = false;
volatile bool note_flag = false;
volatile bool note_selected_flag = false;

float get_freq_safe(bool *valid_flag); // prototype since im lazy
double octave_difference(double f1, double f2);

void adc_irq_handler()
{
    uint16_t adc_value = adc_fifo_get();

    if (prev_adc_value < THRESHOLD && adc_value >= THRESHOLD)
    {
        uint64_t current_time = time_us_64();

        if (last_edge_time != 0)
        {
            uint64_t period = current_time - last_edge_time;
            if (period > MIN_PERIOD_US)
            {
                // Remove the old value from sum
                period_sum -= periods[period_index];

                // Save new period
                periods[period_index] = period;

                // Add the new value into sum
                period_sum += period;

                // Advance index
                period_index = (period_index + 1) % NUM_PERIODS;

                if (periods_collected < NUM_PERIODS)
                {
                    periods_collected++;
                }

                // Calculate average period
                float avg_period = (float)period_sum / periods_collected;

                mutex_enter_blocking(&freq_mutex);
                measured_frequency = 1e6f / avg_period;
                if (periods_collected == NUM_PERIODS)
                {
                    freq_valid = true;
                }
                mutex_exit(&freq_mutex);
            }
        }

        last_edge_time = current_time;
    }

    prev_adc_value = adc_value;
}

void initADC()
{
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0); // Select ADC input 0 (GPIO 26)

    adc_fifo_setup(
        true,  // Write each result to the FIFO
        false, // No DMA request
        1,     // 1 sample before asserting irq
        false, // No error bit
        false  // No 8-bit shift (keep full 12-bit)
    );

    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler); // Set the ADC FIFO interrupt handler
    irq_set_enabled(ADC_IRQ_FIFO, true);                      // Enable the ADC FIFO interrupt
    adc_irq_set_enabled(true);                                // Enable the ADC interrupt

    adc_run(true); // Start the ADC
}

bool MCP4725_setVoltage(i2c_inst_t *i2c_port, uint8_t i2c_addr, bool writeEEprom, uint16_t value, uint32_t speed)
{

    i2c_set_baudrate(i2c_port, speed); // Set the I2C speed

    uint8_t buffer[3];

    if (writeEEprom)
    {
        buffer[0] = MCP4725_CMD_WRITEDACEEPROM; // Command to write to EEPROM
    }
    else
    {
        buffer[0] = MCP4725_CMD_WRITEDAC; // Command to write to DAC register
    }

    buffer[1] = value / 16;        // upper 4 bits of the 12 bit value
    buffer[2] = (value % 16) << 4; // Shift the last 4 bits to the left
                                   // Attempt to write all 3 bytes
    int written = i2c_write_blocking(i2c_port, i2c_addr, buffer, 3, false);
    return (written == 3);
}

bool setVoltageNoEEPROM(uint8_t i2c_addr, uint16_t value)
{
    return MCP4725_setVoltage(I2C_PORT, i2c_addr, false, value, 1000 * 1000);
}

bool setVoltageSafePWM(uint8_t i2c_addr, uint16_t value) // MAX input voltage for pwm is 2.55v. Don't want the voltage to exceed that.
{
    if (value > 3165)
    {
        value = 3165; // Limit the value to 3165
    }
    return MCP4725_setVoltage(I2C_PORT, i2c_addr, true, value, 1000 * 1000);
}

void initI2C(uint16_t baudrate)
{
    // I2C Initialisation. Using it at 400kHz.
    i2c_init(I2C_PORT, baudrate * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void initGPIO()
{
    // Initialize Buttons
    gpio_init(Button1);
    gpio_set_dir(Button1, GPIO_IN);
    gpio_pull_up(Button1);

    gpio_init(Button2);
    gpio_set_dir(Button2, GPIO_IN);
    gpio_pull_up(Button2);

    gpio_init(Button3);
    gpio_set_dir(Button3, GPIO_IN);
    gpio_pull_up(Button3);

    gpio_init(Button4);
    gpio_set_dir(Button4, GPIO_IN);
    gpio_pull_up(Button4);

    // Initialize LEDs
    gpio_init(LED1);
    gpio_set_dir(LED1, GPIO_OUT);

    gpio_init(LED2);
    gpio_set_dir(LED2, GPIO_OUT);

    gpio_init(LED3);
    gpio_set_dir(LED3, GPIO_OUT);
}

void button_callback(uint gpio, uint32_t events)
{
    if (gpio == Button1)
    {
        mutex_enter_blocking(&button_mutex);
        tune_flag = true;
        mutex_exit(&button_mutex);
        gpio_put(LED1, 1);
        printf("Button 1 pressed!\n");
    }
    else if (gpio == Button2)
    {
        note_letter_index = (note_letter_index + 1) % NUM_NOTE_LETTERS;

        mutex_enter_blocking(&note_mutex);
        current_note.letter = note_letters[note_letter_index];
        note_flag = true;
        mutex_exit(&note_mutex);

        // printf("Letter changed to: %c\n", current_note.letter);
    }
    else if (gpio == Button3)
    {
        note_octave = (note_octave + 1) % NUM_TUNE_POINTS;
        mutex_enter_blocking(&note_mutex);
        current_note.octave = note_octave;
        note_flag = true;
        mutex_exit(&note_mutex);

        // printf("Octave changed to: %d\n", current_note.octave);
    }
    else if (gpio == Button4)
    {
        mutex_enter_blocking(&note_selected_mutex);
        note_selected_flag = true;
        mutex_exit(&note_selected_mutex);
        // printf("Button 4 pressed!\n");
    }
}

void initButtonInterrupts()
{
    gpio_set_irq_enabled_with_callback(Button1, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled(Button2, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(Button3, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(Button4, GPIO_IRQ_EDGE_FALL, true);
}

void fill_actual(struct tunable *tunable, float initial_freq)
{
    if (tunable == NULL || tunable->noteData == NULL)
        return;

    double dac_step = tunable->dac_size / (tunable->num_tune_points - 1);
    printf("Dac step = %.2f\n", dac_step);

    for (int i = 0; i < tunable->num_tune_points; i++)
    {
        // populating expected frequency
        tunable->noteData[i].expected_frequency = initial_freq * pow(2.0, i);
        // populating expected DAC values based on frequencies
        tunable->noteData[i].actual_dac = i * dac_step;
        // just in case
        if (tunable->noteData[i].actual_dac > 4095)
        {
            tunable->noteData[i].actual_dac = 4095;
        }
    }

    // tunable->noteData[0].expected_dac = tunable->noteData[0].expected_dac + DAC_OFFSET_MAX_MIN;
    // tunable->noteData[tunable->num_tune_points].expected_dac = tunable->noteData[tunable->num_tune_points].expected_dac - DAC_OFFSET_MAX_MIN;
}

uint32_t dac_value_for_freq(double target_freq, double dac_delta_per_octave, double ref_freq)
{
    double ratio = target_freq / ref_freq;
    double dac_value = log2l(ratio) * (dac_delta_per_octave);
    if (dac_value > 4095)
    {
        dac_value = 4094;
    }
    return dac_value + 0.5;
}

double octave_difference(double f1, double f2)
{
    return log2(f1 / f2); // Calculate the octave difference
}

void get_actual_dac_values(struct tunable *tunable, double initial_freq)
{
    bool is_valid_freq;
    double freq;
    double dac_per_octave = tunable->dac_size / tunable->num_tune_points;

    for (int i = 0; i < tunable->num_tune_points; i++)
    {
        // Set the actual dac frequency
        setVoltageNoEEPROM(CV_ADDRESS, tunable->noteData[i].actual_dac);
        printf("Setting voltage to actual value %d\n", tunable->noteData[i].actual_dac);
        sleep_ms(100);
        do
        {
            freq = get_freq_safe(&is_valid_freq);
            if (!is_valid_freq)
            {
                printf("Frequency not yet valid: %.2f\n", freq);
                sleep_ms(25); // small delay to avoid spamming
            }
        } while (!is_valid_freq);
        // save the frequency for finding the expected dac value later

        tunable->noteData[i].actual_frequency = freq;
    }

    // At each point, find the expected dac value
    for (int i = 0; i < tunable->num_tune_points; i++)
    {
        double oct_delta = octave_difference(tunable->noteData[i].actual_frequency, initial_freq);
        printf("calculating octave delta with frequency %.2f\n", tunable->noteData[i].actual_frequency);
        double expected_dac = oct_delta * dac_per_octave;
        printf("For octave [%d], Dac step = [%.2f], octave delta = [%.2f], Expected Dac Value = [%.2f]\n", i, dac_per_octave, oct_delta, expected_dac);
        tunable->noteData[i].expected_dac = expected_dac;
    }
}

int create_correction_table(struct tunable *tunable, double initial_frequency)
{
    if (tunable == NULL || tunable->num_tune_points < 2)
    {
        printf("Error, not enough points to interpolate\n");
        return 1;
    }

    double octave_delta = octave_difference(tunable->noteData[1].actual_frequency, tunable->noteData[0].actual_frequency);
    double dac_step = (tunable->dac_size / (tunable->num_tune_points - 1)) / octave_delta;
    printf("Dac step = %.2f\n", dac_step);
    double low_freq_dac_value = dac_value_for_freq(initial_frequency, dac_step, tunable->noteData[0].actual_frequency);

    tunable->noteData[0].actual_dac = 0.0;
    tunable->noteData[0].expected_dac = 0.0 + low_freq_dac_value;

    double slope;
    int tuned_index;
    for (int i = 0; i < tunable->num_tune_points - 1; i++)
    {
        slope = (tunable->noteData[i + 1].actual_dac - tunable->noteData[i].actual_dac) /
                (tunable->noteData[i + 1].expected_dac - tunable->noteData[i].expected_dac);
        printf("Slope for range [%d] = %.2f\n", i, slope);

        for (tuned_index = (int)(tunable->noteData[i].expected_dac + 0.5);
             tuned_index < (int)(tunable->noteData[i + 1].expected_dac - 0.5) &&
             tuned_index < tunable->dac_size;
             ++tuned_index)
        {
            int x = tuned_index - (int)(tunable->noteData[i].expected_dac + 0.5);
            int y = (int)(slope * x + tunable->noteData[i].actual_dac + 0.5);
            tunable->dac_calibration_table[tuned_index] = y > 0 ? y : 0;
        }
    }

    // fill in remaining elements of table from tuned_index-1 onward.
    // this is essentially resolution thrown away.
    printf("correcting remaining from index: %d", tuned_index);
    for (int i = tuned_index - 1; i < tunable->dac_size; ++i)
    {
        tunable->dac_calibration_table[i] = tunable->dac_calibration_table[tuned_index - 1];
    }

    return 0;
}

void core1_entry()
{
    initADC();
    initGPIO();
    initButtonInterrupts();
    while (1)
    {
        tight_loop_contents();
    }
}

void init_mutex()
{
    mutex_init(&freq_mutex);
    mutex_init(&button_mutex);
    mutex_init(&note_mutex);
    mutex_init(&note_selected_mutex);
}

float get_freq_safe(bool *valid_flag)
{
    float freq;
    mutex_enter_blocking(&freq_mutex);
    freq = measured_frequency;
    *valid_flag = freq_valid;
    mutex_exit(&freq_mutex);
    return freq;
}

void print_tuned_notes(struct tunable *tn)
{
    if (tn == NULL)
    {
        printf("Error: Tunable struct pointer is NULL.\n");
        return;
    }

    printf("Tunable Structure Contents:\n");
    printf("---------------------------\n");

    // Print noteData (the array of noteData structs)
    printf("noteData:\n");
    if (tn->noteData != NULL)
    {
        for (int i = 0; i < tn->num_tune_points; i++)
        {
            printf("  [%d] expected_frequency: %f, actual frequency: %f, expected_dac: %f, actual_dac: %u\n",
                   i, tn->noteData[i].expected_frequency,
                   tn->noteData[i].actual_frequency,
                   tn->noteData[i].expected_dac,
                   tn->noteData[i].actual_dac);
        }
    }
    else
    {
        printf("  noteData pointer is NULL.\n");
    }

    // Print num_tune_points
    printf("num_tune_points: %d\n", tn->num_tune_points);

    // Print dac_calibration_table (the pointer and the first few values)
    printf("dac_calibration_table: %p\n", (void *)tn->dac_calibration_table); // Print the pointer address
    if (tn->dac_calibration_table != NULL)
    {
        printf("  First few calibration table entries:\n");
        for (int i = 0; i < 5 && i < tn->dac_size; i++) // Print only a few entries
        {
            printf("   [%d]: %u\n", i, tn->dac_calibration_table[i]);
        }
    }
    else
    {
        printf(" dac_calibration_table pointer is NULL.\n");
    }

    // Print dac_size
    printf("dac_size: %u\n", tn->dac_size);
}

void check_flags(bool *tune_start, bool *note_change, bool *note_selected)
{
    mutex_enter_blocking(&button_mutex);
    *tune_start = tune_flag;
    mutex_exit(&button_mutex);

    mutex_enter_blocking(&note_mutex);
    *note_change = note_flag;
    mutex_exit(&note_mutex);

    mutex_enter_blocking(&note_selected_mutex);
    *note_selected = note_selected_flag;
    mutex_exit(&note_selected_mutex);
}

void print_and_update_current_note(struct note *local_note)
{
    mutex_enter_blocking(&note_mutex);
    *local_note = current_note;
    mutex_exit(&note_mutex);

    printf("Selected note: %c%d\n", local_note->letter, local_note->octave);
}

double note_to_frequency(struct note *note)
{
    if (!note)
        return 0.0;

    const char letters[] = {'C', 'D', 'E', 'F', 'G', 'A', 'B'};
    const int semitones_from_C[] = {0, 2, 4, 5, 7, 9, 11}; // C to B

    const int A4_INDEX = 5; // 'A' is at index 5
    const int A4_OCTAVE = 4;
    const double A4_FREQ = 440.0;

    int semitone_offset = -1;
    for (int i = 0; i < 7; i++)
    {
        if (note->letter == letters[i])
        {
            semitone_offset = semitones_from_C[i];
            break;
        }
    }

    if (semitone_offset == -1 || note->octave < 0 || note->octave > 8)
    {
        printf("Invalid note: %c%d\n", note->letter, note->octave);
        return 0.0;
    }

    int total_semitones = semitone_offset + 12 * note->octave;
    int semitones_from_A4 = total_semitones - (semitones_from_C[A4_INDEX] + 12 * A4_OCTAVE);

    return A4_FREQ * pow(2.0, semitones_from_A4 / 12.0);
}

int main()
{
    stdio_init_all();
    sleep_ms(4000);

    init_mutex();
    multicore_launch_core1(core1_entry);

    initI2C(400);

    fill_actual(&tuned_notes, initial_note_frequency);
    /*
    for (int i = 0; i < tuned_notes.num_tune_points; i++)
    {
        printf("expected frequency = %.2f, expected DAC value = %.2f\n",
               tuned_notes.noteData[i].expected_frequency,
               tuned_notes.noteData[i].actual_dac);
    }
    */
    get_actual_dac_values(&tuned_notes, initial_note_frequency);
    create_correction_table(&tuned_notes, initial_note_frequency);
    // print_tuned_notes(&tuned_notes);


    bool is_valid_frequency = false;
    float freq = 0;
    //Local flags for checking tuning. 
    bool start_tune = false; 
    bool change_note = false;
    bool note_selected = false;
    struct note core_zero_note = {'A', 4};
    double note_select_frequency;
    int dac_select_value;
    double dac_delta_per_octave = tuned_notes.dac_size / tuned_notes.num_tune_points;


    while (1)
    {
        check_flags(&start_tune, &change_note, &note_selected);

        if (start_tune)
        {
            get_actual_dac_values(&tuned_notes, initial_note_frequency);
            create_correction_table(&tuned_notes, initial_note_frequency);
            mutex_enter_blocking(&button_mutex);
            tune_flag = false;
            mutex_exit(&button_mutex);
            gpio_put(LED1, 0);
        }

        if (change_note)
        {
            print_and_update_current_note(&core_zero_note);
            mutex_enter_blocking(&note_mutex);
            note_flag = false;
            mutex_exit(&note_mutex);
        }
        if (note_selected)
        {
            note_select_frequency = note_to_frequency(&core_zero_note);

            printf("the current note is %c%d\n", core_zero_note.letter, core_zero_note.octave);
            printf("The frequency of the note selected is %.2f\n", note_select_frequency);

            dac_select_value = dac_value_for_freq(note_select_frequency, dac_delta_per_octave, initial_note_frequency);

            printf("Setting DAC to selected value...\n");

            setVoltageNoEEPROM(CV_ADDRESS, calibration_table[dac_select_value]);
            sleep_ms(100);

            do
            {
                freq = get_freq_safe(&is_valid_frequency);
                if (!is_valid_frequency)
                {
                    printf("Frequency not yet valid: %.2f\n", freq);
                    sleep_ms(25); 
                }
            } while (!is_valid_frequency);

            printf("The measured frequency is : %.2f\n", freq);

            mutex_enter_blocking(&note_selected_mutex);
            note_selected_flag = false;
            mutex_exit(&note_selected_mutex);
        }
    }
}
