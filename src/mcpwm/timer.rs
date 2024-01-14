/*
* Implements the timer related stuff for the ESP-IDF's MCPWM binds for rust since esp_idf_svc does not include it yet.
*
* REQ-TMR-000: Create all the necessary "rusty" structures to bind the esp_idf version. The C
* header file for the mcpwm types can be found here:
* https://github.com/espressif/esp-idf/blob/master/components/esp_driver_mcpwm/include/driver/mcpwm_types.h
*
* REQ-TMR-001: Create a couple of functions and implement the traits to sort of mask the raw C
* functions.
* */

use std::{marker::PhantomData, ptr};

use crate::mcpwm::Group;
use esp_idf_svc::{
    hal::prelude::{FromValueType, Hertz},
    sys::{
        esp, mcpwm_del_timer, mcpwm_new_timer, mcpwm_timer_config_t,
        mcpwm_timer_config_t__bindgen_ty_1, mcpwm_timer_count_mode_t,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_DOWN,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_PAUSE,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_UP,
        mcpwm_timer_count_mode_t_MCPWM_TIMER_COUNT_MODE_UP_DOWN, mcpwm_timer_enable,
        mcpwm_timer_handle_t, mcpwm_timer_start_stop,
        mcpwm_timer_start_stop_cmd_t_MCPWM_TIMER_START_NO_STOP,
        soc_periph_mcpwm_timer_clk_src_t_MCPWM_TIMER_CLK_SRC_DEFAULT, EspError,
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimerConfig {
    resolution: Hertz,
    period_ticks: u16,
    count_mode: CountMode,
    // TODO:
    // on_full: FF,
    // on_empty: FE,
    // on_stop: FS,
}

pub struct TIMER<const N: u8, G: Group> {
    _ptr: PhantomData<*const ()>,
    _group: PhantomData<G>,
}

pub struct TimerDriver<const N: u8, G: Group> {
    _group: G,
    handle: mcpwm_timer_handle_t,
    _timer: TIMER<N, G>,

    period_ticks: u32,
    period_peak: u16,
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

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            resolution: 80.MHz().into(),
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
    /// Calculate the frequency as
    /// `frequency = resolution / period_ticks`
    ///
    /// For a resolution of 80MHz and a period_ticks of 8_000:
    /// `10kHz = 80MHz / 8_000`
    #[must_use]
    pub fn period_ticks(mut self, period_ticks: u16) -> Self {
        self.period_ticks = period_ticks;
        return self;
    }

    #[must_use]
    pub fn count_mode(mut self, counter_mode: CountMode) -> Self {
        self.count_mode = counter_mode;
        return self;
    }
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

impl<const N: u8, G: Group> TIMER<N, G> {
    #[inline(always)]
    pub unsafe fn new() -> Self {
        Self {
            _ptr: PhantomData,
            _group: PhantomData,
        }
    }
}

impl<const N: u8, G: Group> TimerDriver<N, G> {
    pub fn new(timer: TIMER<N, G>, config: TimerConfig) -> Result<Self, EspError> {
        let mut flags: mcpwm_timer_config_t__bindgen_ty_1 = Default::default();

        // What should these be set to?
        flags.set_update_period_on_empty(1);
        flags.set_update_period_on_sync(0);

        let cfg = mcpwm_timer_config_t {
            group_id: G::ID,
            clk_src: soc_periph_mcpwm_timer_clk_src_t_MCPWM_TIMER_CLK_SRC_DEFAULT,
            resolution_hz: config.resolution.0,
            count_mode: config.count_mode.into(),
            period_ticks: config.period_ticks.into(),
            intr_priority: 0, // Maybe let this be user configurable?
            flags,
        };
        let mut handle: mcpwm_timer_handle_t = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_timer(&cfg, &mut handle))?;
        }

        // I think this has to be called before mcpwm_timer_enable
        // mcpwm_timer_register_event_callbacks()
        unsafe {
            esp!(mcpwm_timer_enable(handle))?;
            esp!(mcpwm_timer_start_stop(
                handle,
                mcpwm_timer_start_stop_cmd_t_MCPWM_TIMER_START_NO_STOP
            ))?;
        }

        let period_peak = if config.count_mode == CountMode::UpDown {
            (cfg.period_ticks / 2).try_into().unwrap()
        } else {
            cfg.period_ticks.try_into().unwrap()
        };

        return Ok(Self {
            _group: G::default(),
            handle,
            _timer: timer,
            period_ticks: cfg.period_ticks,
            period_peak,
        });
    }

    pub fn get_period_ticks(&self) -> u32 {
        self.period_ticks
    }

    pub fn get_period_peak(&self) -> u16 {
        self.period_peak
    }

    pub fn timer(&self) -> mcpwm_timer_handle_t {
        self.handle
    }
}

// Not sure if this should be done in TimerConnection instead.
impl<const N: u8, G: Group> Drop for TimerDriver<N, G> {
    fn drop(&mut self) {
        unsafe {
            esp!(mcpwm_del_timer(self.handle)).unwrap();
        }
    }
}
