#include <hardware/adc.h>
#include <hardware/flash.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include <CRC32.h>

#include <string.h>

#include <atomic>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#define DEFAULT_I2C_ADDR 0x37

/////////////////////////////////////////////////////////////////////

void init_i2c_as_slave();
void check_commands();
bool load_config();
void save_config(bool);

/////////////////////////////////////////////////////////////////////

constexpr uint8_t i2c_scl = 3;
constexpr uint8_t i2c_sda = 2;
constexpr size_t num_gpio = 40;
constexpr size_t max_avg_win = 32;
constexpr size_t adc_default_avg = 10;
constexpr size_t adc_default_scale = 65536 / 4096;
const size_t i2c_clock = 100e3;
i2c_inst_t* const i2c_inst = i2c1;

constexpr size_t config_flash_offset = 768 * 1024;

/////////////////////////////////////////////////////////////////////

enum gpio_config_enum : uint8_t
{
    gpio_config_off = 0,
    gpio_config_in = 1,
    gpio_config_in_up = 2,
    gpio_config_in_dn = 3,
    gpio_config_adc = 4,
    //gpio_config_adc_vref = 5, // TODO
};

struct calibration_struct
{
    int pre_offset;
    int scale;
    int post_offset;
};

struct global_config_struct
{
    uint8_t wire_addr;
    gpio_config_enum gpio_config[num_gpio];
    calibration_struct adc_calibration[num_gpio];
    size_t avg_win_sz;
};

/////////////////////////////////////////////////////////////////////

std::atomic<int> core_stop;

global_config_struct config;
std::vector<uint8_t> dig_gpio;
std::vector<uint8_t> adc_gpio;

std::atomic<uint16_t> gpio_values[num_gpio];

uint8_t wire_buffer[32];
uint16_t avg_window[num_gpio][max_avg_win];

uint8_t i2c_rd_addr;
uint16_t i2c_rd_word;

/////////////////////////////////////////////////////////////////////

int add_to_average(
    int channel,
    int latest_adc_value
)
{
    auto& window = avg_window[channel];

    if (config.avg_win_sz == 0 || config.avg_win_sz == 1)
        return latest_adc_value;

    int avg = latest_adc_value;

    for (size_t i = 0; i < config.avg_win_sz - 1; i++)
    {
        auto curr = window[i + 1];
        window[i] = curr;
        avg += curr;
    }

    window[config.avg_win_sz - 1] = latest_adc_value;
    avg /= (int)config.avg_win_sz;

    return avg;
}

void gpio_reader()
{
	multicore_lockout_victim_init();

    std::cout << "GPIO reader task started." << std::endl;

    while (core_stop.load(std::memory_order_relaxed) == 0)
    {
        for (size_t i = 0; i < adc_gpio.size(); i++)
        {
            const auto adc_pin = adc_gpio[i];

            switch (adc_pin)
            {
            case 26: adc_select_input(0); break;
            case 27: adc_select_input(1); break;
            case 28: adc_select_input(2); break;
            case 29: adc_select_input(3); break;
            }

            const auto& cal = config.adc_calibration[i];
            int adc_value = adc_read();
            adc_value = add_to_average(i, adc_value);
            adc_value += cal.pre_offset;
            adc_value *= cal.scale;
            adc_value += cal.post_offset;
            gpio_values[adc_pin].store((uint16_t)adc_value,
                std::memory_order_relaxed);
        }

        uint16_t curr_word = 0;

        for (size_t i = 0; i < dig_gpio.size(); i++)
        {
            const auto pin_bit = i % 16;
            const auto value = gpio_get(dig_gpio[i]);

            if (value == false)
                curr_word |= (uint16_t)(1 << pin_bit);

            if ((pin_bit == 15) || (i == dig_gpio.size() - 1))
            {
                // Use the earlier digital pins to store the state of the
                // entire set of bits
                const auto gpio = dig_gpio[i / 16];

                gpio_values[gpio].store(curr_word,
                    std::memory_order_relaxed);

                curr_word = 0;
            }
        }
    }

    core_stop = 2;
}

int main()
{
    stdio_init_all();
    adc_init();

    // Wait for UART reconnect
    for (int i = 0; i < 10; i++)
    {
        std::cout << ".";
        std::cout.flush();
        sleep_ms(1000);
    }
    std::cout << std::endl << "Hello!" << std::endl;

    if (!load_config())
        save_config(false);

    for (uint8_t i = 0; i < num_gpio; i++)
    {
        switch (config.gpio_config[i])
        {
        case gpio_config_in:
            std::cout << "GPIO " << (int)i << ": Input" << std::endl;
            gpio_init(i);
            gpio_set_dir(i, GPIO_IN);
            dig_gpio.push_back(i);
            break;
        case gpio_config_in_up:
            std::cout << "GPIO " << (int)i << ": Input Pull-Up" << std::endl;
            gpio_init(i);
            gpio_set_dir(i, GPIO_IN);
            gpio_pull_up(i);
            dig_gpio.push_back(i);
            break;
        case gpio_config_in_dn:
            std::cout << "GPIO " << (int)i << ": Input Pull-Down" << std::endl;
            gpio_init(i);
            gpio_set_dir(i, GPIO_IN);
            gpio_pull_down(i);
            dig_gpio.push_back(i);
            break;
        case gpio_config_adc:
            std::cout << "GPIO " << (int)i << ": ADC" << std::endl;
            adc_gpio_init(i);
            adc_gpio.push_back(i);
            break;
        case gpio_config_off:
            break;
        default:
            break;
        }
    }

    std::cout << "Digital GPIOs: " << dig_gpio.size() << std::endl;
    std::cout << "Analog GPIOs: " << adc_gpio.size() << std::endl;
    std::cout << "ADC average samples: " << config.avg_win_sz << std::endl;

    std::cout << "Calibration settings:" << std::endl;
    for (size_t i = 0; i < adc_gpio.size(); i++)
    {
        const auto gpio = adc_gpio[i];
        const auto& cal = config.adc_calibration[gpio];

        std::cout
            << gpio
            << " - PREOFF: " << cal.pre_offset
            << ", SCALE: "   << cal.scale
            << ", POSTOFF: " << cal.post_offset;
    }

    std::cout
        << "Initializing I2C addr=" << config.wire_addr
        << ",sda=" << (int)i2c_sda << ",scl=" << (int)i2c_scl
        << "..." << std::endl;

    init_i2c_as_slave();

    std::cout << "Starting ADC task..." << std::endl;
    core_stop = 0;
	multicore_launch_core1(gpio_reader);

    while (true)
    {
        check_commands();
        sleep_ms(1);
    }

	return 0;
}

// Taken from https://github.com/raspberrypi/pico-sdk/issues/508
void i2c_irq_handler()
{
    auto i2c_hw = i2c_inst->hw;

    uint32_t status = i2c_hw->intr_stat;
    uint32_t value;

    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS)
    {
        value = i2c_hw->data_cmd;

        if (value & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS)
            i2c_rd_addr = (uint8_t)(value & I2C_IC_DATA_CMD_BITS);
    }

    /* Check if the I2C master is requesting data from us. */
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS)
    {
        const auto word_idx  = i2c_rd_addr / 2;
        const auto word_byte = i2c_rd_addr % 2;

        i2c_rd_addr++;

        if (word_byte == 0)
        {
            const auto num_adc_words = adc_gpio.size();
            const auto num_dig_words = (dig_gpio.size() == 0) ? 0 :
                ((dig_gpio.size() - 1) / 16 + 1);
            const auto num_words = num_adc_words + num_dig_words;

            if (word_idx >= num_words)
            {
                i2c_rd_word = 0;
            }
            else
            {
                uint8_t gpio;

                if (word_idx < num_adc_words)
                {
                    gpio = adc_gpio[word_idx];
                }
                else
                {
                    gpio = dig_gpio[word_idx - num_adc_words];
                }

                i2c_rd_word = gpio_values[gpio].load(
                    std::memory_order_relaxed);
            }
        }

        /* Write the data. */
        i2c_hw->data_cmd = (i2c_rd_word >> (8 * word_byte)) & 0xFF;

        /* Clear the interrupt. */
        i2c_hw->clr_rd_req;
    }
}

// Taken from https://github.com/raspberrypi/pico-sdk/issues/508
void init_i2c_as_slave()
{
    i2c_rd_addr = 0;
    i2c_rd_word = 0;

    i2c_init(i2c_inst, i2c_clock);
    i2c_set_slave_mode(i2c_inst, true, config.wire_addr);
    gpio_set_function(i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda);
    gpio_pull_up(i2c_scl);

    i2c_inst->hw->intr_mask =
        I2C_IC_INTR_MASK_M_RD_REQ_BITS |
        I2C_IC_INTR_MASK_M_RX_FULL_BITS;

    const auto irq = i2c_hw_index(i2c_inst) == 0 ? I2C0_IRQ : I2C1_IRQ;

    irq_set_exclusive_handler(irq, i2c_irq_handler);
    irq_set_enabled(irq, true);
}

/////////////////////////////////////////////////////////////////////

bool try_read_int(int& value)
{
    std::cin >> value;

    std::cout << value << '/';

    return true;
}

bool try_read_gpio_idx(int& gpio)
{
    if (!try_read_int(gpio))
        return false;

    if (gpio < 0 || gpio >= (int)num_gpio)
    {
        std::cout << "Invalid GPIO: " << gpio << std::endl;

        return false;
    }

    return true;
}

bool try_read_mode(gpio_config_enum& mode)
{
    int imode;

    if (!try_read_int(imode))
        return false;

    mode = (gpio_config_enum)imode;

    switch (imode)
    {
    case gpio_config_off:
    case gpio_config_in:
    case gpio_config_in_up:
    case gpio_config_in_dn:
    case gpio_config_adc:
        return true;
    default:
        std::cout << "Invalid GPIO mode: " << imode << std::endl;
        return false;
    }
}

void process_c_command()
{
    int gpio = 0;

    // Calibration: gpio pre_offset scale post_offset
    if (!try_read_gpio_idx(gpio) ||
        !try_read_int(config.adc_calibration[gpio].pre_offset) ||
        !try_read_int(config.adc_calibration[gpio].scale) ||
        !try_read_int(config.adc_calibration[gpio].post_offset))
    {
        std::cout
            << std::endl
            << "ADC [C]alibration: GPIO pre_off scale post_off"
            << std::endl;

        return;
    }

    const auto& cal = config.adc_calibration[gpio];

    std::cout << std::endl
              << "ADC " << gpio << ":" << std::endl
              << " PREOFF : " << cal.pre_offset << std::endl
              << " SCALE  : " << cal.scale << std::endl
              << " POSTOFF: " << cal.post_offset << std::endl;
}

void process_m_command()
{
    int gpio = 0;
    gpio_config_enum mode = gpio_config_off;

    // Set GPIO mode
    if (!try_read_gpio_idx(gpio) ||
        !try_read_mode(mode))
    {
        std::cout << std::endl << "Set GPIO [M]ode: GPIO mode" << std::endl;

        return;
    }

    config.gpio_config[gpio] = mode;

    std::cout << std::endl << "GPIO " << gpio << " mode set to " << std::endl;

    switch (mode)
    {
    case gpio_config_off:
        std::cout << "off" << std::endl;
        break;
    case gpio_config_in:
        std::cout << "digital" << std::endl;
        break;
    case gpio_config_in_up:
        std::cout << "digital with internal pull-up" << std::endl;
        break;
    case gpio_config_in_dn:
        std::cout << "digital with internal pull-down" << std::endl;
        break;
    case gpio_config_adc:
        std::cout << "analog" << std::endl;
        break;
    default:
        // Should not happen...
        std::cout << "?" << std::endl;
        break;
    }
}

void process_p_command()
{
    std::cout << std::endl;

    if (adc_gpio.size() == 0 && dig_gpio.size() == 0)
    {
        std::cout << "No input configured!" << std::endl;

        return;
    }

    for (size_t i = 0; i < adc_gpio.size(); i++)
    {
        const auto gpio = adc_gpio[i];
        const auto adc_value = gpio_values[gpio].load(
            std::memory_order_relaxed);

        std::cout << "ADC " << i << ": " << adc_value << std::endl;
    }

    const size_t num_dig_words = (dig_gpio.size() == 0) ? 0 :
        ((dig_gpio.size() - 1) / 16 + 1);

    if (num_dig_words > 0)
    {
        std::stringstream ss;
        for (size_t i = 0; i < num_dig_words; i++)
        {
            const auto gpio = dig_gpio[i];
            const auto dig_value = gpio_values[gpio].load(
                std::memory_order_relaxed);

            ss << std::hex << dig_value;
        }
        std::cout << "Digital: " << ss.str() << std::endl;
    }
}

void process_v_command()
{
    int window_sz;

    if (!try_read_int(window_sz))
    {
        std::cout << std::endl << "ADC a[V]erage samples: count" << std::endl;

        return;
    }

    if (window_sz < 0 || window_sz > (int)max_avg_win)
    {
        std::cout << std::endl
                  << "Invalid average samples " << window_sz
                  << ", max " << max_avg_win << std::endl;

        return;
    }

    config.avg_win_sz = window_sz;

    std::cout << std::endl
              << "ADC average samples set to " << config.avg_win_sz
              << std::endl;
}

void process_w_command()
{
    std::cout << std::endl;

    save_config(true);
}

void check_commands()
{
    typedef decltype(std::cin)::char_type char_type;
    typedef std::char_traits<char_type> char_traits;

    if (std::cin.peek() == char_traits::eof())
        return;

    char command;

    std::cin >> command;

    std::cout << command << '/';

    switch (command)
    {
    case 0  : std::cout << "Serial error!" << std::endl; break;
    case 'c': case 'C': process_c_command(); break;
    case 'm': case 'M': process_m_command(); break;
    case 'p': case 'P': process_p_command(); break;
    case 'v': case 'V': process_v_command(); break;
    case 'w': case 'W': process_w_command(); break;
    case 'r': case 'R': watchdog_reboot(0, SRAM_END, 10); break;
    default:
        std::cout << "Unknown command " << command << std::endl;
        break;
    }
}

void print_hash(const char* name, uint32_t hash)
{
    std::cout << name << " hash: ";

    std::stringstream ss;

    ss << std::setw(8) << std::setfill('0') << std::hex << hash;

    std::cout << ss.str() << std::endl;
}

bool load_config()
{
    bool config_loaded = false;

    auto p_mem = (const uint8_t*)(XIP_BASE + config_flash_offset);
    auto p_config = (const global_config_struct*)p_mem;
    auto p_hash = (const uint32_t*)(p_mem + sizeof(config));

    CRC32 crc;
    crc.update(p_mem, sizeof(config));
    const auto hash = crc.finalize();

    print_hash("Stored", *p_hash);
    print_hash("Computed", hash);

    if (hash != *p_hash)
    {
        std::cout << "Hash mismatch!" << std::endl;
    }
    else
    {
        config = *p_config;
        config_loaded = true;
    }

    if (!config_loaded)
    {
        // Create the default configuration

        config.wire_addr = DEFAULT_I2C_ADDR;
        config.avg_win_sz = adc_default_avg;

        for (auto& gpio: config.gpio_config)
            gpio = gpio_config_off;

        for (auto& cal: config.adc_calibration)
        {
            cal.scale = adc_default_scale;
            cal.pre_offset = 0;
            cal.post_offset = 0;
        }

        // Enable digital input + pull up on GPIO17. This
        // is the same pin as the default S2 (START).
        config.gpio_config[17] = gpio_config_in_up;
    }

    return config_loaded;
}

void save_config(bool stop_core)
{
    static_assert(sizeof(config) + sizeof(uint32_t) <= FLASH_SECTOR_SIZE);

    if (stop_core)
    {
        std::cout << "Stopping core 1..." << std::endl;

        core_stop = 1;

        while (core_stop.load(std::memory_order_relaxed) != 2) ;
    }

    std::vector<uint8_t> data(FLASH_SECTOR_SIZE);

    memcpy(&data[0], &config, sizeof(config));

    auto p_hash = (uint32_t*)(&data[0] + sizeof(config));

    CRC32 crc;
    crc.update(&data[0], sizeof(config));
    *p_hash = crc.finalize();

    print_hash("Configuration", *p_hash);

    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(config_flash_offset, FLASH_SECTOR_SIZE);
    flash_range_program(config_flash_offset, &data[0], FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    auto p_mem = (const uint8_t*)(XIP_BASE + config_flash_offset);
    auto p_config = (const global_config_struct*)p_mem;
    auto p_hash2 = (const uint32_t*)(p_mem + sizeof(config));

    print_hash("Saved", *p_hash2);

    std::cout << "Written configuration file!" << std::endl;
}
