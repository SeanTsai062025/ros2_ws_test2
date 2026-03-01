/*
 * fix_pwm0_clock.c  –  Raspberry Pi 5 RP1 clk_pwm0 initialiser
 *
 * On many Pi 5 / Ubuntu kernel combinations the device-tree overlay that
 * should configure clk_pwm0 never fires, so the clock's parent and
 * divider are left at their reset defaults (rate = 0).  The kernel PWM
 * driver therefore refuses to program any period/duty.
 *
 * clk_pwm1 *is* initialised (used by the fan controller), so we simply
 * copy its CTRL / DIV_INT / DIV_FRAC / SEL values into the clk_pwm0
 * register set.  The result is clk_pwm0 running at the same ~6.144 MHz
 * rate as clk_pwm1, which gives ample resolution for 50 Hz servo signals.
 *
 * Must run as root (needs /dev/mem).
 *
 * Build:  gcc -O2 -o fix_pwm0_clock fix_pwm0_clock.c
 * Run:    sudo ./fix_pwm0_clock
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

/* RP1 clock-controller physical base (from /proc/iomem 1f00018000) */
#define RP1_CLK_BASE        0x1f00018000ULL
#define MAP_SIZE            0x1000UL

/* Register offsets inside the clock block (from drivers/clk/clk-rp1.c) */
#define CLK_PWM0_CTRL       0x074
#define CLK_PWM0_DIV_INT    0x078
#define CLK_PWM0_DIV_FRAC   0x07c
#define CLK_PWM0_SEL        0x080

#define CLK_PWM1_CTRL       0x084
#define CLK_PWM1_DIV_INT    0x088
#define CLK_PWM1_DIV_FRAC   0x08c
#define CLK_PWM1_SEL        0x090

#define CLK_CTRL_ENABLE     (1u << 11)

static volatile uint32_t *map_phys(int fd, uint64_t phys)
{
    uint64_t page = phys & ~(uint64_t)(MAP_SIZE - 1);
    void *m = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                   fd, (off_t)page);
    if (m == MAP_FAILED) return NULL;
    return (volatile uint32_t *)((uint8_t *)m + (phys - page));
}

static inline uint32_t r32(volatile uint32_t *b, uint32_t off)
{ return *(volatile uint32_t *)((uint8_t *)b + off); }

static inline void w32(volatile uint32_t *b, uint32_t off, uint32_t v)
{ *(volatile uint32_t *)((uint8_t *)b + off) = v; }

int main(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }

    volatile uint32_t *clk = map_phys(fd, RP1_CLK_BASE);
    if (!clk) { perror("mmap clk"); close(fd); return 1; }

    /* Read the working pwm1 settings */
    uint32_t c1_ctrl = r32(clk, CLK_PWM1_CTRL);
    uint32_t c1_dint = r32(clk, CLK_PWM1_DIV_INT);
    uint32_t c1_dfrc = r32(clk, CLK_PWM1_DIV_FRAC);
    uint32_t c1_sel  = r32(clk, CLK_PWM1_SEL);

    uint32_t c0_ctrl = r32(clk, CLK_PWM0_CTRL);

    printf("pwm1 ctrl=0x%08x  div_int=0x%08x  div_frac=0x%08x  sel=0x%08x\n",
           c1_ctrl, c1_dint, c1_dfrc, c1_sel);
    printf("pwm0 ctrl=0x%08x  (before)\n", c0_ctrl);

    if ((c0_ctrl & CLK_CTRL_ENABLE) && r32(clk, CLK_PWM0_DIV_INT) > 0) {
        printf("clk_pwm0 already configured (enabled, div>0) – nothing to do.\n");
        close(fd);
        return 0;
    }

    /* Disable → set sel → set dividers → copy ctrl → enable */
    w32(clk, CLK_PWM0_CTRL, c0_ctrl & ~CLK_CTRL_ENABLE);
    usleep(10);
    w32(clk, CLK_PWM0_SEL,      c1_sel);
    w32(clk, CLK_PWM0_DIV_INT,  c1_dint);
    w32(clk, CLK_PWM0_DIV_FRAC, c1_dfrc);
    w32(clk, CLK_PWM0_CTRL, c1_ctrl & ~CLK_CTRL_ENABLE);
    usleep(10);
    w32(clk, CLK_PWM0_CTRL, c1_ctrl | CLK_CTRL_ENABLE);
    usleep(100);

    uint32_t verify = r32(clk, CLK_PWM0_CTRL);
    printf("pwm0 ctrl=0x%08x  (after)\n", verify);
    printf("%s\n", (verify & CLK_CTRL_ENABLE) ? "OK" : "FAIL");

    close(fd);
    return (verify & CLK_CTRL_ENABLE) ? 0 : 1;
}
