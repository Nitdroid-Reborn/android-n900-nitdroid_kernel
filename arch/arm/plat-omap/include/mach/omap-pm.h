/*
 * omap-pm.h - OMAP power management interface
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Copyright (C) 2008 Nokia Corporation
 * Paul Walmsley
 *
 * Interface developed by (in alphabetical order): Karthik Dasu, Jouni
 * Högander, Tony Lindgren, Rajendra Nayak, Sakari Poussa,
 * Veeramanikandan Raju, Anand Sawant, Igor Stoppa, Paul Walmsley,
 * Richard Woodruff
 */

#ifndef ASM_ARM_ARCH_OMAP_OMAP_PM_H
#define ASM_ARM_ARCH_OMAP_OMAP_PM_H

#include <linux/device.h>
#include <linux/cpufreq.h>

#include "powerdomain.h"

/**
 * struct omap_opp - clock frequency-to-OPP ID table for DSP, MPU
 * @rate: target clock rate
 * @opp_id: OPP ID
 * @min_vdd: minimum VDD1 voltage (in millivolts) for this OPP
 *
 * Operating performance point data.  Can vary by OMAP chip and board.
 */
struct omap_opp {
	unsigned long rate;
	u8 opp_id;
	u16 vsel;
};

extern struct omap_opp *mpu_opps;
extern struct omap_opp *dsp_opps;
extern struct omap_opp *l3_opps;

/*
 * agent_id values for use with omap_pm_set_min_bus_tput():
 *
 * OCP_INITIATOR_AGENT is only valid for devices that can act as
 * initiators -- it represents the device's L3 interconnect
 * connection.  OCP_TARGET_AGENT represents the device's L4
 * interconnect connection.
 */
#define OCP_TARGET_AGENT		1
#define OCP_INITIATOR_AGENT		2

/**
 * omap_pm_if_early_init - OMAP PM init code called before clock fw init
 * @mpu_opp_table: array ptr to struct omap_opp for MPU
 * @dsp_opp_table: array ptr to struct omap_opp for DSP
 * @l3_opp_table : array ptr to struct omap_opp for CORE
 *
 * Initialize anything that must be configured before the clock
 * framework starts.  The "_if_" is to avoid name collisions with the
 * PM idle-loop code.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline int __init omap_pm_if_early_init(struct omap_opp *mpu_opp_table,
				 struct omap_opp *dsp_opp_table,
				 struct omap_opp *l3_opp_table) { return 0; }
#else
int __init omap_pm_if_early_init(struct omap_opp *mpu_opp_table,
				 struct omap_opp *dsp_opp_table,
				 struct omap_opp *l3_opp_table);
#endif

/**
 * omap_pm_if_init - OMAP PM init code called after clock fw init
 *
 * The main initialization code.  OPP tables are passed in here.  The
 * "_if_" is to avoid name collisions with the PM idle-loop code.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline int __init omap_pm_if_init(void) { return 0; }
#else
int __init omap_pm_if_init(void);
#endif

/**
 * omap_pm_if_exit - OMAP PM exit code
 *
 * Exit code; currently unused.  The "_if_" is to avoid name
 * collisions with the PM idle-loop code.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_if_exit(void) { }
#else
void omap_pm_if_exit(void);
#endif

/*
 * Device-driver-originated constraints (via board-*.c files, platform_data)
 */


/**
 * omap_pm_set_max_mpu_wakeup_lat - set the maximum MPU wakeup latency
 * @dev: struct device * requesting the constraint
 * @t: maximum MPU wakeup latency in microseconds
 *
 * Request that the maximum interrupt latency for the MPU to be no
 * greater than 't' microseconds. "Interrupt latency" in this case is
 * defined as the elapsed time from the occurrence of a hardware or
 * timer interrupt to the time when the device driver's interrupt
 * service routine has been entered by the MPU.
 *
 * It is intended that underlying PM code will use this information to
 * determine what power state to put the MPU powerdomain into, and
 * possibly the CORE powerdomain as well, since interrupt handling
 * code currently runs from SDRAM.  Advanced PM or board*.c code may
 * also configure interrupt controller priorities, OCP bus priorities,
 * CPU speed(s), etc.
 *
 * This function will not affect device wakeup latency, e.g., time
 * elapsed from when a device driver enables a hardware device with
 * clk_enable(), to when the device is ready for register access or
 * other use.  To control this device wakeup latency, use
 * set_max_dev_wakeup_lat()
 *
 * Multiple calls to set_max_mpu_wakeup_lat() will replace the
 * previous t value.  To remove the latency target for the MPU, call
 * with t = -1.
 *
 * No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_set_max_mpu_wakeup_lat(struct device *dev, long t) { }
#else
void omap_pm_set_max_mpu_wakeup_lat(struct device *dev, long t);
#endif


/**
 * omap_pm_set_min_bus_tput - set minimum bus throughput needed by device
 * @dev: struct device * requesting the constraint
 * @tbus_id: interconnect to operate on (OCP_{INITIATOR,TARGET}_AGENT)
 * @r: minimum throughput (in KiB/s)
 *
 * Request that the minimum data throughput on the OCP interconnect
 * attached to device 'dev' interconnect agent 'tbus_id' be no less
 * than 'r' KiB/s.
 *
 * It is expected that the OMAP PM or bus code will use this
 * information to set the interconnect clock to run at the lowest
 * possible speed that satisfies all current system users.  The PM or
 * bus code will adjust the estimate based on its model of the bus, so
 * device driver authors should attempt to specify an accurate
 * quantity for their device use case, and let the PM or bus code
 * overestimate the numbers as necessary to handle request/response
 * latency, other competing users on the system, etc.  On OMAP2/3, if
 * a driver requests a minimum L4 interconnect speed constraint, the
 * code will also need to add an minimum L3 interconnect speed
 * constraint,
 *
 * Multiple calls to set_min_bus_tput() will replace the previous rate
 * value for this device.  To remove the interconnect throughput
 * restriction for this device, call with r = 0.
 *
 * No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_set_min_bus_tput(struct device *dev, u8 agent_id, unsigned long r) { }
#else
void omap_pm_set_min_bus_tput(struct device *dev, u8 agent_id, unsigned long r);
#endif

/**
 * omap_pm_set_min_mpu_freq - set minimum MPU frequency needed by device
 * @dev: struct device * requesting the constraint
 * @r: minimum MPU frequency (in Hz)
 *
 * Request that the minimum MPU frequency be no less than 'r' Hz.
 *
 * Multiple calls to set_min_mpu_freq() will replace the previous rate value
 * for this device. To remove the frequency restriction for this device,
 * call with r = 0.
 *
 * No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_set_min_mpu_freq(struct device *dev,
		unsigned long r) { }
#else
void omap_pm_set_min_mpu_freq(struct device *dev, unsigned long r);
#endif


/**
 * omap_pm_set_max_dev_wakeup_lat - set the maximum device enable latency
 * @dev: struct device *
 * @t: maximum device wakeup latency in microseconds
 *
 * Request that the maximum amount of time necessary for a device to
 * become accessible after its clocks are enabled should be no greater
 * than 't' microseconds.  Specifically, this represents the time from
 * when a device driver enables device clocks with clk_enable(), to
 * when the register reads and writes on the device will succeed.
 * This function should be called before clk_disable() is called,
 * since the power state transition decision may be made during
 * clk_disable().
 *
 * It is intended that underlying PM code will use this information to
 * determine what power state to put the powerdomain enclosing this
 * device into.
 *
 * Multiple calls to set_max_dev_wakeup_lat() will replace the
 * previous wakeup latency values for this device.  To remove the wakeup
 * latency restriction for this device, call with t = -1.
 *
 * No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_set_max_dev_wakeup_lat(struct device *dev, long t) { }
#else
void omap_pm_set_max_dev_wakeup_lat(struct device *dev, long t);
#endif


/**
 * omap_pm_set_max_sdma_lat - set the maximum system DMA transfer start latency
 * @dev: struct device *
 * @t: maximum DMA transfer start latency in microseconds
 *
 * Request that the maximum system DMA transfer start latency for this
 * device 'dev' should be no greater than 't' microseconds.  "DMA
 * transfer start latency" here is defined as the elapsed time from
 * when a device (e.g., McBSP) requests that a system DMA transfer
 * start or continue, to the time at which data starts to flow into
 * that device from the system DMA controller.
 *
 * It is intended that underlying PM code will use this information to
 * determine what power state to put the CORE powerdomain into.
 *
 * Since system DMA transfers may not involve the MPU, this function
 * will not affect MPU wakeup latency.  Use set_max_cpu_lat() to do
 * so.  Similarly, this function will not affect device wakeup latency
 * -- use set_max_dev_wakeup_lat() to affect that.
 *
 * Multiple calls to set_max_sdma_lat() will replace the previous t
 * value for this device.  To remove the maximum DMA latency for this
 * device, call with t = -1.
 *
 * No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_set_max_sdma_lat(struct device *dev, long t) { }
#else
void omap_pm_set_max_sdma_lat(struct device *dev, long t);
#endif


/*
 * DSP Bridge-specific constraints
 */

/**
 * omap_pm_dsp_get_opp_table - get OPP->DSP clock frequency table
 *
 * Intended for use by DSPBridge.  Returns an array of OPP->DSP clock
 * frequency entries.  The final item in the array should have .rate =
 * .opp_id = 0.
 */
const struct omap_opp *omap_pm_dsp_get_opp_table(void);

/**
 * omap_pm_dsp_set_min_opp - receive desired OPP target ID from DSP Bridge
 * @opp_id: target DSP OPP ID
 *
 * Set a minimum OPP ID for the DSP.  This is intended to be called
 * only from the DSP Bridge MPU-side driver.  Unfortunately, the only
 * information that code receives from the DSP/BIOS load estimator is the
 * target OPP ID; hence, this interface.  No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_dsp_set_min_opp(u8 opp_id) { }
#else
void omap_pm_dsp_set_min_opp(u8 opp_id);
#endif

/**
 * omap_pm_dsp_get_opp - report the current DSP OPP ID
 *
 * Report the current OPP for the DSP.  Since on OMAP3, the DSP and
 * MPU share a single voltage domain, the OPP ID returned back may
 * represent a higher DSP speed than the OPP requested via
 * omap_pm_dsp_set_min_opp().
 *
 * Returns the current VDD1 OPP ID, or 0 upon error.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline u8 omap_pm_dsp_get_opp(void) { return 0; }
#else
u8 omap_pm_dsp_get_opp(void);
#endif


/*
 * CPUFreq-originated constraint
 *
 * In the future, this should be handled by custom OPP clocktype
 * functions.
 */

/**
 * omap_pm_cpu_get_freq_table - return a cpufreq_frequency_table array ptr
 *
 * Provide a frequency table usable by CPUFreq for the current chip/board.
 * Returns a pointer to a struct cpufreq_frequency_table array or NULL
 * upon error.
 */
struct cpufreq_frequency_table **omap_pm_cpu_get_freq_table(void);

/**
 * omap_pm_cpu_set_freq - set the current minimum MPU frequency
 * @f: MPU frequency in Hz
 *
 * Set the current minimum CPU frequency.  The actual CPU frequency
 * used could end up higher if the DSP requested a higher OPP.
 * Intended to be called by plat-omap/cpu_omap.c:omap_target().  No
 * return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_cpu_set_freq(unsigned long f) { }
#else
void omap_pm_cpu_set_freq(unsigned long f);
#endif

/**
 * omap_pm_cpu_get_freq - report the current CPU frequency
 *
 * Returns the current MPU frequency, or 0 upon error.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline unsigned long omap_pm_cpu_get_freq(void) { return 0; }
#else
unsigned long omap_pm_cpu_get_freq(void);
#endif


/*
 * Device context loss tracking
 */

/**
 * omap_pm_get_dev_context_loss_count - return count of times dev has lost ctx
 * @dev: struct device *
 *
 * This function returns the number of times that the device @dev has
 * lost its internal context.  This generally occurs on a powerdomain
 * transition to OFF.  Drivers use this as an optimization to avoid restoring
 * context if the device hasn't lost it.  To use, drivers should initially
 * call this in their context save functions and store the result.  Early in
 * the driver's context restore function, the driver should call this function
 * again, and compare the result to the stored counter.  If they differ, the
 * driver must restore device context.   If the number of context losses
 * exceeds the maximum positive integer, the function will wrap to 0 and
 * continue counting.  Returns the number of context losses for this device,
 * or -EINVAL upon error.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline int omap_pm_get_dev_context_loss_count(struct device *dev) { return 0; }
#else
int omap_pm_get_dev_context_loss_count(struct device *dev);
#endif


/*
 * Powerdomain usecounting hooks
 */

/**
 * omap_pm_pwrdm_active - indicate that a power domain has become active
 * @pwrdm: struct powerdomain *
 *
 * Notify the OMAP PM layer that the power domain 'pwrdm' has become active,
 * presumably due to a device driver enabling an underlying clock.  This
 * function is intended to be called by the clockdomain code, not by drivers.
 * No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_pwrdm_active(struct powerdomain *pwrdm) { }
#else
void omap_pm_pwrdm_active(struct powerdomain *pwrdm);
#endif


/**
 * omap_pm_pwrdm_inactive - indicate that a power domain has become inactive
 * @pwrdm: struct powerdomain *
 *
 * Notify the OMAP PM layer that the power domain 'pwrdm' has become
 * inactive, presumably due to a device driver disabling an underlying
 * clock.  This function is intended to be called by the clockdomain
 * code, not by drivers.  No return value.
 */
#ifdef CONFIG_OMAP_PM_NONE
static inline void omap_pm_pwrdm_inactive(struct powerdomain *pwrdm) { }
#else
void omap_pm_pwrdm_inactive(struct powerdomain *pwrdm);
#endif

#endif
