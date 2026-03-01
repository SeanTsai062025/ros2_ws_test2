/*
 * servo_hw_pwm_node.cpp
 *
 * Production-ready ROS 2 (Jazzy) node that drives an MG996R servo on a
 * Raspberry Pi 5 using RP1 hardware PWM via direct register access.
 *
 * Why direct registers instead of /sys/class/pwm?
 *   On many Pi 5 + Ubuntu kernel 6.8 combinations the device-tree overlay
 *   that should initialise clk_pwm0 never fires, leaving its rate at 0.
 *   The sysfs PWM driver therefore refuses to set period/duty.  We solve
 *   this by:
 *     1. Copying clk_pwm1's (working) clock configuration into clk_pwm0.
 *     2. Programming the PWM channel registers directly.
 *   This gives jitter-free 50 Hz hardware PWM – no software timing loops.
 *
 * Subscribes to: /servo_angle  (std_msgs/msg/Float64, degrees 0-180)
 *
 * Parameters:
 *   gpio_pin           – RP1 GPIO number (default 18, physical pin 12)
 *   min_pulse_us       – pulse width at 0°   (default 500 µs)
 *   max_pulse_us       – pulse width at 180° (default 2500 µs)
 *   pwm_frequency_hz   – PWM frequency        (default 50 Hz)
 *   initial_angle      – angle to drive on startup, <0 = idle (default -1)
 *
 * Build:
 *   colcon build --packages-select servo_hw_pwm
 *
 * Run (requires root for /dev/mem):
 *   sudo -E env "PATH=$PATH" \
 *       ros2 run servo_hw_pwm servo_hw_pwm_node
 */

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// ─── RP1 register addresses (from /proc/iomem & kernel sources) ────────────
static constexpr uint64_t RP1_CLK_BASE  = 0x1f00018000ULL;  // clocks@18000
static constexpr uint64_t RP1_PWM0_BASE = 0x1f00098000ULL;  // pwm@98000
static constexpr uint64_t RP1_GPIO_BASE = 0x1f000d0000ULL;  // gpio@d0000

static constexpr size_t MAP_SIZE = 0x1000;

// ─── Clock register offsets (drivers/clk/clk-rp1.c) ───────────────────────
static constexpr uint32_t CLK_PWM0_CTRL     = 0x074;
static constexpr uint32_t CLK_PWM0_DIV_INT  = 0x078;
static constexpr uint32_t CLK_PWM0_DIV_FRAC = 0x07c;
static constexpr uint32_t CLK_PWM0_SEL      = 0x080;
static constexpr uint32_t CLK_PWM1_CTRL     = 0x084;
static constexpr uint32_t CLK_PWM1_DIV_INT  = 0x088;
static constexpr uint32_t CLK_PWM1_DIV_FRAC = 0x08c;
static constexpr uint32_t CLK_PWM1_SEL      = 0x090;
static constexpr uint32_t CLK_CTRL_ENABLE   = 1u << 11;

// ─── PWM register offsets (drivers/pwm/pwm-rp1.c) ─────────────────────────
static constexpr uint32_t PWM_GLOBAL_CTRL       = 0x000;
static constexpr uint32_t PWM_CHANNEL_CTRL(uint32_t ch)
{ return 0x014 + ch * 16; }
static constexpr uint32_t PWM_RANGE(uint32_t ch)
{ return 0x018 + ch * 16; }
static constexpr uint32_t PWM_DUTY(uint32_t ch)
{ return 0x020 + ch * 16; }

static constexpr uint32_t PWM_CHAN_DEFAULT = (1u << 8) | (1u << 0);  // FIFO_POP_MASK | trailing-edge M/S
static constexpr uint32_t PWM_SET_UPDATE   = 1u << 31;

// ─── GPIO register offsets (drivers/pinctrl/pinctrl-rp1.c) ────────────────
static constexpr uint32_t RP1_GPIO_CTRL            = 0x004;
static constexpr uint32_t RP1_GPIO_CTRL_FUNCSEL_MASK = 0x1f;
static constexpr uint32_t RP1_GPIO_CTRL_OEOVER_LSB  = 14;
static constexpr uint32_t RP1_GPIO_CTRL_OEOVER_MASK = 0x0000c000;
static constexpr uint32_t RP1_GPIO_CTRL_OUTOVER_LSB = 12;
static constexpr uint32_t RP1_GPIO_CTRL_OUTOVER_MASK= 0x00003000;

// RP1 pin function-select values (fsel indices) for GPIO 18:
//   alt0=spi1, alt1=dpi, alt2=i2s0, alt3=pwm0, alt4=i2s1, alt5=gpio, ...
static constexpr uint32_t RP1_FSEL_PWM0 = 3;   // alt3

// Per-GPIO register stride: STATUS(0x00) + CTRL(0x04) = 8 bytes
// Bank 0 (GPIO 0-27) starts at offset 0x0000 in the GPIO block.

// ─── Helper: mmap a physical region ────────────────────────────────────────
static volatile uint32_t *map_phys(int fd, uint64_t phys, const char *tag)
{
    uint64_t page = phys & ~(uint64_t)(MAP_SIZE - 1);
    void *m = mmap(nullptr, MAP_SIZE, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd, static_cast<off_t>(page));
    if (m == MAP_FAILED) {
        throw std::runtime_error(
            std::string("mmap ") + tag + " @ 0x" +
            std::to_string(phys) + ": " + strerror(errno));
    }
    return reinterpret_cast<volatile uint32_t *>(
        static_cast<uint8_t *>(m) + (phys - page));
}

static inline uint32_t r32(volatile uint32_t *base, uint32_t off)
{
    return *reinterpret_cast<volatile uint32_t *>(
        reinterpret_cast<volatile uint8_t *>(base) + off);
}

static inline void w32(volatile uint32_t *base, uint32_t off, uint32_t v)
{
    *reinterpret_cast<volatile uint32_t *>(
        reinterpret_cast<volatile uint8_t *>(base) + off) = v;
}

// ═══════════════════════════════════════════════════════════════════════════
class ServoHwPwmNode : public rclcpp::Node
{
public:
    ServoHwPwmNode()
    : Node("servo_hw_pwm_node")
    {
        // Declare parameters
        declare_parameter("gpio_pin", 18);
        declare_parameter("min_pulse_us", 500.0);
        declare_parameter("max_pulse_us", 2500.0);
        declare_parameter("pwm_frequency_hz", 50.0);
        declare_parameter("initial_angle", -1.0);  // <0 means no output until first msg

        gpio_pin_      = static_cast<uint32_t>(get_parameter("gpio_pin").as_int());
        min_pulse_us_  = get_parameter("min_pulse_us").as_double();
        max_pulse_us_  = get_parameter("max_pulse_us").as_double();
        frequency_hz_  = get_parameter("pwm_frequency_hz").as_double();
        initial_angle_ = get_parameter("initial_angle").as_double();

        if (gpio_pin_ > 27) {
            throw std::invalid_argument(
                "gpio_pin must be 0-27 (RP1 bank 0) for pwm0 on Pi 5");
        }

        pwm_channel_ = gpio_pin_ % 4;

        RCLCPP_INFO(get_logger(),
            "GPIO %u  →  PWM0 channel %u  |  %.0f Hz  |  %.0f-%.0f µs",
            gpio_pin_, pwm_channel_, frequency_hz_,
            min_pulse_us_, max_pulse_us_);

        // Open /dev/mem
        mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            throw std::runtime_error(
                std::string("/dev/mem: ") + strerror(errno) +
                " – run with sudo");
        }

        // Map the three RP1 register blocks we need
        clk_  = map_phys(mem_fd_, RP1_CLK_BASE,  "CLK");
        pwm_  = map_phys(mem_fd_, RP1_PWM0_BASE, "PWM0");
        gpio_ = map_phys(mem_fd_, RP1_GPIO_BASE, "GPIO");

        // 1.  Fix clk_pwm0 (copy settings from the working clk_pwm1)
        fix_pwm0_clock();

        // 2.  Set GPIO pin function to alt3 (pwm0)
        set_gpio_function(gpio_pin_, RP1_FSEL_PWM0);

        // 3.  Derive clock rate and configure PWM channel
        clk_rate_hz_ = derive_clock_rate();
        RCLCPP_INFO(get_logger(), "Derived clock rate: %u Hz", clk_rate_hz_);

        range_ = static_cast<uint32_t>(
            std::round(static_cast<double>(clk_rate_hz_) / frequency_hz_));

        RCLCPP_INFO(get_logger(), "PWM RANGE (ticks per period): %u", range_);

        configure_pwm_channel();

        // 4.  Subscribe
        sub_ = create_subscription<std_msgs::msg::Float64>(
            "servo_angle", rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
                on_angle(msg->data);
            });

        RCLCPP_INFO(get_logger(), "Ready – listening on /servo_angle");
    }

    ~ServoHwPwmNode() override
    {
        disable_pwm_channel();
        if (mem_fd_ >= 0) close(mem_fd_);
        RCLCPP_INFO(get_logger(), "PWM disabled, cleanup done.");
    }

private:
    // ── Parameters ──────────────────────────────────────────────────────
    uint32_t gpio_pin_{18};
    uint32_t pwm_channel_{2};
    double   min_pulse_us_{500.0};
    double   max_pulse_us_{2500.0};
    double   frequency_hz_{50.0};
    double   initial_angle_{-1.0};
    uint32_t clk_rate_hz_{0};
    uint32_t range_{0};

    // ── Hardware ────────────────────────────────────────────────────────
    int mem_fd_{-1};
    volatile uint32_t *clk_{nullptr};
    volatile uint32_t *pwm_{nullptr};
    volatile uint32_t *gpio_{nullptr};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

    // ── Fix clk_pwm0 by copying clk_pwm1's register values ─────────────
    void fix_pwm0_clock()
    {
        uint32_t c1_ctrl = r32(clk_, CLK_PWM1_CTRL);
        uint32_t c0_ctrl = r32(clk_, CLK_PWM0_CTRL);

        if ((c0_ctrl & CLK_CTRL_ENABLE) && r32(clk_, CLK_PWM0_DIV_INT) > 0) {
            RCLCPP_INFO(get_logger(), "clk_pwm0 already configured (ctrl=0x%08x).", c0_ctrl);
            return;
        }

        uint32_t c1_dint = r32(clk_, CLK_PWM1_DIV_INT);
        uint32_t c1_dfrc = r32(clk_, CLK_PWM1_DIV_FRAC);
        uint32_t c1_sel  = r32(clk_, CLK_PWM1_SEL);

        RCLCPP_INFO(get_logger(),
            "Fixing clk_pwm0: copying pwm1 ctrl=0x%08x div=%u frac=0x%08x sel=0x%08x",
            c1_ctrl, c1_dint, c1_dfrc, c1_sel);

        // Disable → set source sel → set dividers → copy ctrl → enable
        w32(clk_, CLK_PWM0_CTRL, c0_ctrl & ~CLK_CTRL_ENABLE);
        usleep(10);
        w32(clk_, CLK_PWM0_SEL,      c1_sel);
        w32(clk_, CLK_PWM0_DIV_INT,  c1_dint);
        w32(clk_, CLK_PWM0_DIV_FRAC, c1_dfrc);
        w32(clk_, CLK_PWM0_CTRL, c1_ctrl & ~CLK_CTRL_ENABLE);
        usleep(10);
        w32(clk_, CLK_PWM0_CTRL, c1_ctrl | CLK_CTRL_ENABLE);
        usleep(100);

        uint32_t verify = r32(clk_, CLK_PWM0_CTRL);
        if (!(verify & CLK_CTRL_ENABLE)) {
            throw std::runtime_error("Failed to enable clk_pwm0");
        }
        RCLCPP_INFO(get_logger(), "clk_pwm0 enabled (ctrl=0x%08x)", verify);
    }

    // ── Derive the clock rate from the divider registers ────────────────
    //    Parent is xosc @ 50 MHz.  Rate = 50 MHz / (div_int + div_frac/2^16)
    //    But the kernel uses 16 frac bits shifted from a 32-bit register:
    //      effective_div = div_int + (div_frac >> 16) / 65536.0
    uint32_t derive_clock_rate()
    {
        uint32_t dint  = r32(clk_, CLK_PWM0_DIV_INT);
        uint32_t dfrac = r32(clk_, CLK_PWM0_DIV_FRAC);
        // The fraction is stored in the upper 16 bits of the 32-bit register,
        // as per CLK_DIV_FRAC_BITS = 16 in the kernel.
        double div = static_cast<double>(dint) +
                     static_cast<double>(dfrac >> 16) / 65536.0;
        if (div < 1.0) div = 1.0;
        // xosc = 50 MHz
        return static_cast<uint32_t>(std::round(50000000.0 / div));
    }

    // ── Set the GPIO pin function-select register ───────────────────────
    void set_gpio_function(uint32_t gpio, uint32_t fsel)
    {
        // Bank 0 (GPIO 0-27) base offset = 0x0000
        // Per-pin stride = 8 bytes (STATUS @ +0, CTRL @ +4)
        uint32_t ctrl_off = gpio * 8 + RP1_GPIO_CTRL;

        uint32_t ctrl = r32(gpio_, ctrl_off);

        // Clear FUNCSEL [4:0], OUTOVER [13:12], OEOVER [15:14]
        ctrl &= ~RP1_GPIO_CTRL_FUNCSEL_MASK;
        ctrl &= ~RP1_GPIO_CTRL_OUTOVER_MASK;
        ctrl &= ~RP1_GPIO_CTRL_OEOVER_MASK;

        // Set FUNCSEL to the requested alt function
        ctrl |= (fsel & RP1_GPIO_CTRL_FUNCSEL_MASK);
        // OUTOVER = 0 (peripheral), OEOVER = 0 (peripheral) – already cleared

        w32(gpio_, ctrl_off, ctrl);

        RCLCPP_INFO(get_logger(),
            "GPIO %u CTRL = 0x%08x (fsel=%u → pwm0 alt3)", gpio, ctrl, fsel);
    }

    // ── Configure the PWM channel ──────────────────────────────────────
    void configure_pwm_channel()
    {
        // Set channel mode (M/S, no FIFO pop mask)
        w32(pwm_, PWM_CHANNEL_CTRL(pwm_channel_), PWM_CHAN_DEFAULT);

        // Set range (period in clock ticks)
        w32(pwm_, PWM_RANGE(pwm_channel_), range_);

        if (initial_angle_ >= 0.0) {
            // Drive servo to the requested initial position
            double angle = std::clamp(initial_angle_, 0.0, 180.0);
            uint32_t init_duty = angle_to_ticks(angle);
            w32(pwm_, PWM_DUTY(pwm_channel_), init_duty);

            RCLCPP_INFO(get_logger(),
                "PWM ch%u: initial angle %.1f° → duty=%u ticks",
                pwm_channel_, angle, init_duty);
        } else {
            // No initial output — set duty to 0 (no pulse)
            w32(pwm_, PWM_DUTY(pwm_channel_), 0);

            RCLCPP_INFO(get_logger(),
                "PWM ch%u: no initial angle — idle until first /servo_angle msg",
                pwm_channel_);
        }

        // Enable this channel in the global control register + SET_UPDATE
        uint32_t gc = r32(pwm_, PWM_GLOBAL_CTRL);
        gc |= (1u << pwm_channel_);   // enable channel
        gc |= PWM_SET_UPDATE;
        w32(pwm_, PWM_GLOBAL_CTRL, gc);

        RCLCPP_INFO(get_logger(),
            "PWM ch%u enabled: range=%u, global_ctrl=0x%08x",
            pwm_channel_, range_, gc);
    }

    // ── Disable the PWM channel ────────────────────────────────────────
    void disable_pwm_channel()
    {
        if (!pwm_) return;
        uint32_t gc = r32(pwm_, PWM_GLOBAL_CTRL);
        gc &= ~(1u << pwm_channel_);
        gc |= PWM_SET_UPDATE;
        w32(pwm_, PWM_GLOBAL_CTRL, gc);
    }

    // ── Convert angle (0-180°) to PWM ticks ────────────────────────────
    uint32_t angle_to_ticks(double angle_deg) const
    {
        angle_deg = std::clamp(angle_deg, 0.0, 180.0);

        // Interpolate pulse width in µs
        double pulse_us = min_pulse_us_ +
            (angle_deg / 180.0) * (max_pulse_us_ - min_pulse_us_);

        // Convert µs → ticks:  ticks = pulse_us * clk_rate / 1e6
        double ticks = pulse_us * static_cast<double>(clk_rate_hz_) / 1.0e6;

        return static_cast<uint32_t>(std::round(ticks));
    }

    // ── Topic callback ─────────────────────────────────────────────────
    void on_angle(double angle_deg)
    {
        double clamped = std::clamp(angle_deg, 0.0, 180.0);
        uint32_t duty = angle_to_ticks(clamped);

        w32(pwm_, PWM_DUTY(pwm_channel_), duty);

        // Latch the new duty with SET_UPDATE
        uint32_t gc = r32(pwm_, PWM_GLOBAL_CTRL);
        gc |= PWM_SET_UPDATE;
        w32(pwm_, PWM_GLOBAL_CTRL, gc);

        RCLCPP_DEBUG(get_logger(), "angle=%.1f° → duty=%u ticks", clamped, duty);
    }
};

// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ServoHwPwmNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("servo_hw_pwm"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
