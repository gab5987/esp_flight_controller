/*
* Theres no implementation of the ESP-IDFs mcpwm C module in the esp_idf_svc crate, so this file
* implements it with a few limitations and not as general as it is on the C version.
*
* Heavily based on the ledc implementation: https://github.com/esp-rs/esp-idf-hal/blob/master/src/ledc.rs
* */

use core::ffi;
use esp_idf_svc::{
    hal::prelude::Hertz,
    sys::{
        esp, mcpwm_cmpr_handle_t, mcpwm_comparator_config_t,
        mcpwm_comparator_config_t__bindgen_ty_1, mcpwm_gen_compare_event_action_t,
        mcpwm_gen_handle_t, mcpwm_gen_timer_event_action_t, mcpwm_generator_action_t,
        mcpwm_generator_action_t_MCPWM_GEN_ACTION_HIGH,
        mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP,
        mcpwm_generator_action_t_MCPWM_GEN_ACTION_LOW,
        mcpwm_generator_action_t_MCPWM_GEN_ACTION_TOGGLE, mcpwm_generator_config_t,
        mcpwm_generator_config_t__bindgen_ty_1, mcpwm_generator_set_action_on_compare_event,
        mcpwm_generator_set_action_on_timer_event, mcpwm_new_comparator, mcpwm_new_generator,
        mcpwm_new_operator, mcpwm_new_timer, mcpwm_oper_handle_t, mcpwm_operator_config_t,
        mcpwm_operator_config_t__bindgen_ty_1, mcpwm_operator_connect_timer, mcpwm_timer_config_t,
        mcpwm_timer_config_t__bindgen_ty_1, mcpwm_timer_count_mode_t,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_DOWN,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_PAUSE,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_UP,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP, mcpwm_timer_enable,
        mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY, mcpwm_timer_handle_t, mcpwm_timer_start_stop,
        mcpwm_timer_start_stop_cmd_t_MCPWM_TIMER_START_NO_STOP,
        soc_periph_mcpwm_timer_clk_src_t_MCPWM_TIMER_CLK_SRC_DEFAULT, EspError,
    },
};
use std::ptr;

#[derive(Default)]
pub struct Group0;

#[cfg(not(esp32c6))]
#[derive(Default)]
pub struct Group1;

pub type Duty = u16;

// This was called `Unit` in IDF < 5.0
pub trait Group: Default {
    const ID: ffi::c_int;
}

impl Group for Group0 {
    const ID: ffi::c_int = 0;
}

#[cfg(not(esp32c6))]
impl Group for Group1 {
    const ID: ffi::c_int = 1;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CountMode {
    /// Edge aligned. The counter will start from its lowest value and increment every clock cycle until the period is reached.
    /// ```
    ///       start, counter = 0                     reset, counter = period
    ///         |                                       |
    ///         |                                       |*--- start, counter = 0
    ///         v <----  duty  ----> .                  v|
    ///         .                    .                  .v
    ///         .--------------------.                  ..----
    ///         |       Active       |                  .|
    ///         |                    |                  .|
    ///         |                    |     Not active   .|
    ///         -                    ---------------------
    /// ```
    Up,

    /// Edge aligned. The counter will start from its highest value, period and decrement every clock cycle until the zero is reached
    /// ```
    ///       start, counter = period                   reset, counter = 0
    ///         |                                         |
    ///         |                                         |*--- start, counter = period
    ///         v                    .                    v|
    ///         .                    . <----  duty  ----> .v
    ///         .                    .--------------------..
    ///         .       Active       |                    |.
    ///         .                    |                    |.
    ///         .     Not active     |      Active        |.
    ///         ----------------------                    ----
    /// ```
    Down,

    /// Symmetric mode. The counter will start from its lowest value and increment every clock cycle until the period is reached
    /// ```
    ///                                             change count dir to decrement, counter = period
    ///       start, counter = 0, incrementing          |                                     change count dir to increment, counter = 0
    ///         |                                       |                                        |
    ///         |                                       |*--- counter = period                   |*----- start, counter = 0, incrementing
    ///         v <----  duty  ----> .                  v|                  . <----  duty  ----> ||
    ///         .                    .                  .v                  .                    vv
    ///         ---------------------.                  ..                  .-------------------------------------------.                  ..                  .--
    ///                 Active       |                  ..                  |        Active                Active       |                  ..                  |
    ///                              |                  ..                  |                                           |                  ..                  |
    ///                              |     Not active   ..    Not active    |                                           |     Not active   ..    Not active    |
    ///                              ----------------------------------------                                           ----------------------------------------
    /// ```
    /// In this mode, the frequency will be half of that specified
    UpDown,

    /// Timer paused
    Pause,
}

impl From<CountMode> for mcpwm_timer_count_mode_t {
    fn from(val: CountMode) -> Self {
        match val {
            //CounterMode::Frozen => mcpwm_counter_type_t_MCPWM_FREEZE_COUNTER,
            CountMode::Up => mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_UP,
            CountMode::Down => mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_DOWN,
            CountMode::UpDown => mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_UP_DOWN,
            CountMode::Pause => mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_PAUSE,
        }
    }
}

pub struct TimerConfig {
    resolution: Hertz,
    count_mode: CountMode,
    period_ticks: u16,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            /* TODO: Find the optimal default resolution/frequency for the project */
            resolution: Hertz::from(80).into(),
            period_ticks: 8_000, // 10kHz
            count_mode: CountMode::Up,
        }
    }
}

impl TimerConfig {
    /*#[must_use]
    pub fn resolution(mut self, resolution: impl Into<Hertz>) -> Self {
        self.resolution = resolution.into();
        self
    }*/

    /// This is inversely proportional to the frequency of the signal
    /// `frequency = resolution / period_ticks`
    #[must_use]
    pub fn period_ticks(mut self, period_ticks: u16) -> Self {
        self.period_ticks = period_ticks;
        self
    }

    #[must_use]
    pub fn count_mode(mut self, counter_mode: CountMode) -> Self {
        self.count_mode = counter_mode;
        self
    }
}

pub struct TimerDriver<G: Group> {
    _group: G,
    period_ticks: u32,
    period_peak: u16,
    handle: mcpwm_timer_handle_t,
    pub oper: mcpwm_oper_handle_t,
}

impl<G: Group> TimerDriver<G> {
    pub fn new(config: TimerConfig) -> Result<Self, EspError> {
        let mut flags: mcpwm_timer_config_t__bindgen_ty_1 = Default::default();

        // What should these be set to?
        flags.set_update_period_on_empty(1);
        flags.set_update_period_on_sync(0);

        let cfg = mcpwm_timer_config_t {
            intr_priority: 0,
            group_id: G::ID,
            clk_src: soc_periph_mcpwm_timer_clk_src_t_MCPWM_TIMER_CLK_SRC_DEFAULT,
            resolution_hz: config.resolution.0,
            count_mode: config.count_mode.into(),
            period_ticks: config.period_ticks.into(),
            flags,
        };
        let mut handle: mcpwm_timer_handle_t = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_timer(&cfg, &mut handle))?;
        }

        let period_peak = if config.count_mode == CountMode::UpDown {
            (cfg.period_ticks / 2).try_into().unwrap()
        } else {
            cfg.period_ticks.try_into().unwrap()
        };

        let mut oper = ptr::null_mut();
        let mut oper_flags: mcpwm_operator_config_t__bindgen_ty_1 = Default::default();

        // What should these be set to?
        oper_flags.set_update_gen_action_on_tez(0);
        oper_flags.set_update_gen_action_on_tep(0);
        oper_flags.set_update_gen_action_on_sync(0);

        oper_flags.set_update_dead_time_on_tez(0);
        oper_flags.set_update_dead_time_on_tep(0);
        oper_flags.set_update_dead_time_on_sync(0);

        let oper_config = mcpwm_operator_config_t {
            group_id: G::ID,
            intr_priority: 0,
            flags: oper_flags,
        };

        unsafe {
            esp!(mcpwm_new_operator(&oper_config, &mut oper))?;
            esp!(mcpwm_operator_connect_timer(oper, handle))?
        }

        Ok(Self {
            _group: G::default(),
            period_ticks: cfg.period_ticks,
            period_peak,
            handle,
            oper,
        })
    }

    /// Enables the timer group.
    /// Do not enable the timers before setting the callbacks
    pub fn enable(&self) -> Result<(), EspError> {
        // This has to be called before mcpwm_timer_enable
        // mcpwm_timer_register_event_callbacks()
        unsafe {
            esp!(mcpwm_timer_enable(self.handle))?;
            esp!(mcpwm_timer_start_stop(
                self.handle,
                mcpwm_timer_start_stop_cmd_t_MCPWM_TIMER_START_NO_STOP
            ))?;
        }

        return Ok(());
    }
}

/* TODO: Perhaps considering do the same for the other generator config fields. */

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeneratorAction {
    /* TODO Come up with a better name since both the comparator and the generator uses this. */
    Nothing,
    SetLow,
    SetHigh,
    Toggle,
}

impl From<GeneratorAction> for mcpwm_generator_action_t {
    fn from(val: GeneratorAction) -> Self {
        match val {
            GeneratorAction::Nothing => mcpwm_generator_action_t_MCPWM_GEN_ACTION_KEEP,
            GeneratorAction::SetLow => mcpwm_generator_action_t_MCPWM_GEN_ACTION_LOW,
            GeneratorAction::SetHigh => mcpwm_generator_action_t_MCPWM_GEN_ACTION_HIGH,
            GeneratorAction::Toggle => mcpwm_generator_action_t_MCPWM_GEN_ACTION_TOGGLE,
        }
    }
}

pub struct ComparatorConfig {
    pwm_pin: i32,
    gen_action: GeneratorAction,
    cmpr_action: GeneratorAction,
}

pub struct Comparator {
    gen: mcpwm_gen_handle_t,
    pub cmpr: mcpwm_cmpr_handle_t,
}

impl Comparator {
    pub fn new<G: Group>(
        timer_op: &TimerDriver<G>,
        config: ComparatorConfig,
    ) -> Result<Self, EspError> {
        // TODO: Create a way to configure the generator
        let gen_flags: mcpwm_generator_config_t__bindgen_ty_1 = Default::default();
        let gen_config = mcpwm_generator_config_t {
            gen_gpio_num: config.pwm_pin,
            flags: gen_flags,
        };

        let mut cmpr_flags: mcpwm_comparator_config_t__bindgen_ty_1 = Default::default();
        cmpr_flags.set_update_cmp_on_tez(1);
        let cmpr_config = mcpwm_comparator_config_t {
            intr_priority: 0,
            flags: cmpr_flags,
        };

        let mut gen = ptr::null_mut();
        let mut cmpr = ptr::null_mut();

        /* Those hardcoded settings is the default ones, and i dont think ill use any other than those. */

        let gen_ev_act = mcpwm_gen_timer_event_action_t {
            direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
            event: mcpwm_timer_event_t_MCPWM_TIMER_EVENT_EMPTY,
            action: config.gen_action.into(),
        };

        let cmpr_ev_action = mcpwm_gen_compare_event_action_t {
            direction: mcpwm_timer_direction_t_MCPWM_TIMER_DIRECTION_UP,
            comparator: cmpr,
            action: config.cmpr_action.into(),
        };

        unsafe {
            esp!(mcpwm_new_generator(timer_op.oper, &gen_config, &mut gen))?;
            esp!(mcpwm_new_comparator(timer_op.oper, &cmpr_config, &mut cmpr))?;

            esp!(mcpwm_generator_set_action_on_timer_event(gen, gen_ev_act))?;
            esp!(mcpwm_generator_set_action_on_compare_event(
                gen,
                cmpr_ev_action
            ))?;
        }

        return Ok(Self { gen, cmpr });
    }

    /* TODO: Implement a way to change the comparator time */
}
