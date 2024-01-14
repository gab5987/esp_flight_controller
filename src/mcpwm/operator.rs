use super::Group;
use std::marker::PhantomData;

use esp_idf_svc::sys::mcpwm_oper_handle_t;

pub struct OPERATOR<const N: u8, G: Group> {
    _ptr: PhantomData<*const ()>,
    _group: PhantomData<G>,
}

pub struct Operator<const N: u8, G: Group> {
    _instance: OPERATOR<N, G>,
    _handle: mcpwm_oper_handle_t,
}

unsafe impl<const N: u8, G: Group> Send for OPERATOR<N, G> {}
