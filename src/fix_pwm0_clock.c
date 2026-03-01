/*
 * fix_pwm0_clock.c
 *
 * Configures the RP1 clk_pwm0 to match clk_pwm1's working settings
 * by directly writing to RP1 clock registers via /dev/mem.
 *
 * clk_pwm1 (working): ctrl=0x11000840, div_int=8, div_frac=0x23550000, sel=1
 * clk_pwm0 (broken):  ctrl=0x01000800, div_int=1, div_frac=0, sel=1
 *
 * The fix: set clk_pwm0's AUXSRC to xosc (same as pwm1), set matching
 * divider values, and enable the clock.
 *
 * Must be run as root (needs /dev/mem access).
 *
 * Build: gcc -O2 -o fix_pwm0_clock fix_pwm0_clock.c
 * Run:   sudo ./fix_pwm0_clock
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

/* RP1 clock controller base address (from /proc/iomem) */
#define RP1_CLK_BASE        0x1f00018000ULL

/* Clock register offsets for PWM0 (from clk-rp1.c) */
#define CLK_PWM0_CTRL       0x00074
#define CLK_PWM0_DIV_INT    0x00078
#define CLK_PWM0_DIV_FRAC   0x0007c
#define CLK_PWM0_SEL        0x00080

/* Clock register offsets for PWM1 (for reading/verification) */
#define CLK_PWM1_CTRL       0x00084
#define CLK_PWM1_DIV_INT    0x00088
#define CLK_PWM1_DIV_FRAC   0x0008c
#define CLK_PWM1_SEL        0x00090

/* Clock control bits */
#define CLK_CTRL_ENABLE     (1 << 11)

#define MAP_SIZE            0x1000

static volatile uint32_t *map_register(int fd, off_t phys_base)
{
    void *mapped = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd, phys_base);
    if (mapped == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }
    return (volatile uint32_t *)mapped;
}

static uint32_t reg_read(volatile uint32_t *base, uint32_t offset)
{
    return base[offset / 4];
}

static void reg_write(volatile uint32_t *base, uint32_t offset, uint32_t val)
{
    base[offset / 4] = val;
}

int main(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem (run as root?)");
        return 1;
    }

    /* Map the RP1 clock register block */
    volatile uint32_t *clk = map_register(fd, RP1_CLK_BASE & ~(MAP_SIZE - 1));
    if (!clk) {
        close(fd);
        return 1;
    }

    /* Adjust pointer for the page offset */
    size_t page_offset = (RP1_CLK_BASE & (MAP_SIZE - 1)) / 4;
    volatile uint32_t *clk_base = clk + page_offset;

    /* Read current PWM1 registers (known working) */
    uint32_t pwm1_ctrl     = reg_read(clk_base, CLK_PWM1_CTRL);
    uint32_t pwm1_div_int  = reg_read(clk_base, CLK_PWM1_DIV_INT);
    uint32_t pwm1_div_frac = reg_read(clk_base, CLK_PWM1_DIV_FRAC);
    uint32_t pwm1_sel      = reg_read(clk_base, CLK_PWM1_SEL);

    printf("PWM1 (working reference):\n");
    printf("  ctrl     = 0x%08x\n", pwm1_ctrl);
    printf("  div_int  = 0x%08x\n", pwm1_div_int);
    printf("  div_frac = 0x%08x\n", pwm1_div_frac);
    printf("  sel      = 0x%08x\n", pwm1_sel);

    /* Read current PWM0 registers (broken) */
    uint32_t pwm0_ctrl     = reg_read(clk_base, CLK_PWM0_CTRL);
    uint32_t pwm0_div_int  = reg_read(clk_base, CLK_PWM0_DIV_INT);
    uint32_t pwm0_div_frac = reg_read(clk_base, CLK_PWM0_DIV_FRAC);
    uint32_t pwm0_sel      = reg_read(clk_base, CLK_PWM0_SEL);

    printf("\nPWM0 (before fix):\n");
    printf("  ctrl     = 0x%08x\n", pwm0_ctrl);
    printf("  div_int  = 0x%08x\n", pwm0_div_int);
    printf("  div_frac = 0x%08x\n", pwm0_div_frac);
    printf("  sel      = 0x%08x\n", pwm0_sel);

    /* Step 1: Disable clk_pwm0 first (clear enable bit) */
    printf("\nDisabling clk_pwm0...\n");
    reg_write(clk_base, CLK_PWM0_CTRL, pwm0_ctrl & ~CLK_CTRL_ENABLE);
    usleep(10);

    /* Step 2: Set divider to match pwm1 */
    printf("Setting divider: int=0x%08x, frac=0x%08x\n", pwm1_div_int, pwm1_div_frac);
    reg_write(clk_base, CLK_PWM0_DIV_INT, pwm1_div_int);
    reg_write(clk_base, CLK_PWM0_DIV_FRAC, pwm1_div_frac);

    /* Step 3: Set AUXSRC and source to match pwm1 (copy ctrl but ensure enable is off) */
    uint32_t new_ctrl = pwm1_ctrl & ~CLK_CTRL_ENABLE;
    printf("Setting ctrl = 0x%08x (same AUXSRC as pwm1, enable off)\n", new_ctrl);
    reg_write(clk_base, CLK_PWM0_CTRL, new_ctrl);
    usleep(10);

    /* Step 4: Enable the clock */
    new_ctrl |= CLK_CTRL_ENABLE;
    printf("Enabling: ctrl = 0x%08x\n", new_ctrl);
    reg_write(clk_base, CLK_PWM0_CTRL, new_ctrl);
    usleep(100);

    /* Read back to verify */
    pwm0_ctrl     = reg_read(clk_base, CLK_PWM0_CTRL);
    pwm0_div_int  = reg_read(clk_base, CLK_PWM0_DIV_INT);
    pwm0_div_frac = reg_read(clk_base, CLK_PWM0_DIV_FRAC);
    pwm0_sel      = reg_read(clk_base, CLK_PWM0_SEL);

    printf("\nPWM0 (after fix):\n");
    printf("  ctrl     = 0x%08x\n", pwm0_ctrl);
    printf("  div_int  = 0x%08x\n", pwm0_div_int);
    printf("  div_frac = 0x%08x\n", pwm0_div_frac);
    printf("  sel      = 0x%08x\n", pwm0_sel);

    int ok = (pwm0_ctrl & CLK_CTRL_ENABLE) != 0;
    printf("\nclk_pwm0 enable bit: %s\n", ok ? "SET (success!)" : "NOT SET (failed)");

    munmap((void *)clk, MAP_SIZE);
    close(fd);

    return ok ? 0 : 1;
}
