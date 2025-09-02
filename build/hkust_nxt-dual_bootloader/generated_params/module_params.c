
/**
 * Empty cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 * The voltage should be chosen above the steep dropoff at 3.5V. A typical
 * lithium battery can only be discharged under high load down to 10% before
 * it drops off to a voltage level damaging the cells.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_V_EMPTY, 3.6);

/**
 * Empty cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 * The voltage should be chosen above the steep dropoff at 3.5V. A typical
 * lithium battery can only be discharged under high load down to 10% before
 * it drops off to a voltage level damaging the cells.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_V_EMPTY, 3.6);

/**
 * Empty cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 * The voltage should be chosen above the steep dropoff at 3.5V. A typical
 * lithium battery can only be discharged under high load down to 10% before
 * it drops off to a voltage level damaging the cells.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_V_EMPTY, 3.6);

/**
 * Full cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered full.
 * For a more accurate estimate set this below the nominal voltage of e.g. 4.2V
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_V_CHARGED, 4.05);

/**
 * Full cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered full.
 * For a more accurate estimate set this below the nominal voltage of e.g. 4.2V
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_V_CHARGED, 4.05);

/**
 * Full cell voltage
 *
 * Defines the voltage where a single cell of the battery is considered full.
 * For a more accurate estimate set this below the nominal voltage of e.g. 4.2V
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @unit V
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_V_CHARGED, 4.05);

/**
 * Explicitly defines the per cell internal resistance for battery 1
 *
 * If non-negative, then this will be used instead of the online estimated internal resistance.
 * 
 *
 * @group Battery Calibration
 * @decimal 4
 * @increment 0.0005
 * @min -1.0
 * @max 0.2
 * @unit Ohm
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_R_INTERNAL, -1.0);

/**
 * Explicitly defines the per cell internal resistance for battery 2
 *
 * If non-negative, then this will be used instead of the online estimated internal resistance.
 * 
 *
 * @group Battery Calibration
 * @decimal 4
 * @increment 0.0005
 * @min -1.0
 * @max 0.2
 * @unit Ohm
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_R_INTERNAL, -1.0);

/**
 * Explicitly defines the per cell internal resistance for battery 3
 *
 * If non-negative, then this will be used instead of the online estimated internal resistance.
 * 
 *
 * @group Battery Calibration
 * @decimal 4
 * @increment 0.0005
 * @min -1.0
 * @max 0.2
 * @unit Ohm
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_R_INTERNAL, -1.0);

/**
 * Number of cells for battery 1.
 *
 * Defines the number of cells the attached battery consists of.
 * 
 *
 * @group Battery Calibration
 * @value 0 Unknown
 * @value 1 1S Battery
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 */
PARAM_DEFINE_INT32(BAT1_N_CELLS, 0);

/**
 * Number of cells for battery 2.
 *
 * Defines the number of cells the attached battery consists of.
 * 
 *
 * @group Battery Calibration
 * @value 0 Unknown
 * @value 1 1S Battery
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 */
PARAM_DEFINE_INT32(BAT2_N_CELLS, 0);

/**
 * Number of cells for battery 3.
 *
 * Defines the number of cells the attached battery consists of.
 * 
 *
 * @group Battery Calibration
 * @value 0 Unknown
 * @value 1 1S Battery
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 */
PARAM_DEFINE_INT32(BAT3_N_CELLS, 0);

/**
 * Battery 1 capacity.
 *
 * Defines the capacity of battery 1 in mAh.
 * 
 *
 * @group Battery Calibration
 * @decimal 0
 * @increment 50
 * @min -1.0
 * @max 100000
 * @unit mAh
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT1_CAPACITY, -1.0);

/**
 * Battery 2 capacity.
 *
 * Defines the capacity of battery 2 in mAh.
 * 
 *
 * @group Battery Calibration
 * @decimal 0
 * @increment 50
 * @min -1.0
 * @max 100000
 * @unit mAh
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT2_CAPACITY, -1.0);

/**
 * Battery 3 capacity.
 *
 * Defines the capacity of battery 3 in mAh.
 * 
 *
 * @group Battery Calibration
 * @decimal 0
 * @increment 50
 * @min -1.0
 * @max 100000
 * @unit mAh
 * @reboot_required True
 */
PARAM_DEFINE_FLOAT(BAT3_CAPACITY, -1.0);

/**
 * Battery 1 monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 * If the value is set to 'ESCs', the battery information are taken from the esc_status message.
 * This requires the ESC to provide both voltage as well as current.
 * 
 *
 * @group Battery Calibration
 * @value -1 Disabled
 * @value 0 Power Module
 * @value 1 External
 * @value 2 ESCs
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT1_SOURCE, 0);

/**
 * Battery 2 monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 * If the value is set to 'ESCs', the battery information are taken from the esc_status message.
 * This requires the ESC to provide both voltage as well as current.
 * 
 *
 * @group Battery Calibration
 * @value -1 Disabled
 * @value 0 Power Module
 * @value 1 External
 * @value 2 ESCs
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT2_SOURCE, -1);

/**
 * Battery 3 monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 * If the value is set to 'ESCs', the battery information are taken from the esc_status message.
 * This requires the ESC to provide both voltage as well as current.
 * 
 *
 * @group Battery Calibration
 * @value -1 Disabled
 * @value 0 Power Module
 * @value 1 External
 * @value 2 ESCs
 * @reboot_required True
 */
PARAM_DEFINE_INT32(BAT3_SOURCE, -1);

/**
 * Low threshold.
 *
 * Sets the threshold when the battery will be reported as low.
 * This has to be higher than the critical threshold.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @min 0.12
 * @max 0.5
 * @unit norm
 */
PARAM_DEFINE_FLOAT(BAT_LOW_THR, 0.15);

/**
 * Critical threshold.
 *
 * Sets the threshold when the battery will be reported as critically low.
 * This has to be lower than the low threshold. This threshold commonly
 * will trigger RTL.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @min 0.05
 * @max 0.5
 * @unit norm
 */
PARAM_DEFINE_FLOAT(BAT_CRIT_THR, 0.07);

/**
 * Emergency threshold.
 *
 * Sets the threshold when the battery will be reported as dangerously low.
 * This has to be lower than the critical threshold. This threshold commonly
 * will trigger landing.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.01
 * @min 0.03
 * @max 0.5
 * @unit norm
 */
PARAM_DEFINE_FLOAT(BAT_EMERGEN_THR, 0.05);

/**
 * Expected battery current in flight.
 *
 * This value is used to initialize the in-flight average current estimation,
 * which in turn is used for estimating remaining flight time and RTL triggering.
 * 
 *
 * @group Battery Calibration
 * @decimal 2
 * @increment 0.1
 * @min 0
 * @max 500
 * @unit A
 */
PARAM_DEFINE_FLOAT(BAT_AVRG_CURRENT, 15);
