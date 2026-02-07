//! This module implements a management layer for the pipes across multiple threads.

use std::sync::mpsc::Sender;
use std::collections::HashMap;

/// Provides easy to use functions to manage the cycle/pipes across multiple threads.
///
/// This provides the ability to easily find free pipes/cycles to associate with a transponder
/// aircraft and assign a specific theta value for use in beamforming. It also allows one to
/// disassociate a pipe when it is no longer needed causing it to run the standard algorithm
/// which at this moment is a random search.
pub struct PipeManagement {
    txs: Vec<Sender<ThreadTxMessage>>,
    /// Maps transponder address or u32 to a pipe.
    addr_to_pipe: HashMap<u32, (usize, usize)>,
    /// Maps each pipe to a transponder address or u32.
    pipe_to_addr: Vec<Option<u32>>,
    /// The total number of threads.
    thread_count: usize,
    /// The total number of pipes per thread.
    pipe_count: usize,
}

impl PipeManagement {
    /// Create a new management structure.
    ///
    /// This is only called once during program initialization.
    pub fn new(thread_count: usize, pipe_count: usize) -> PipeManagement {
        PipeManagement {
            txs: Vec::new(),
            addr_to_pipe: HashMap::new(),
            pipe_to_addr: vec![None; thread_count * pipe_count],
            thread_count: thread_count,
            pipe_count: pipe_count,
        }
    }

    /// Unassigns a pipe assigned to addr if any.
    pub fn unset_addr(&mut self, addr: u32) {
        let mut args: Option<(usize, usize)> = None;

        match self.addr_to_pipe.get(&addr) {
            None => (),
            Some((thread_ndx, pipe_ndx)) => {
                args = Some((*thread_ndx,  *pipe_ndx));
            },
        }

        match args {
            None => (),
            Some((thread_ndx, pipe_ndx)) => {
                self.addr_to_pipe.remove(&addr);
                self.pipe_to_addr[thread_ndx * self.pipe_count + pipe_ndx] = None;
                self.txs[thread_ndx].send(ThreadTxMessage::UnsetWeights(pipe_ndx)).unwrap();
            }
        }
    }

    /// Sets pipe to theta using global pipe index (across all threads).
    ///
    /// If thetas is None this will unset the pipe and it will return to
    /// random.
    ///
    /// Warning: A subsequent call to `unset_addr` with the same `addr` will
    ///          not unset the pipe.
    pub fn set_pipe_to_theta(&mut self, pipe_ndx: usize, addr: u32, thetas: Option<Vec<f32>>) {
        let thread_ndx = pipe_ndx / self.pipe_count;
        let local_pipe_ndx = pipe_ndx - thread_ndx * self.pipe_count;
        match thetas {
            Some(v) => {
                self.pipe_to_addr[pipe_ndx] = Some(addr);
                self.txs[thread_ndx].send(ThreadTxMessage::SetWeights(local_pipe_ndx, v)).unwrap();
            },
            None => {
                self.pipe_to_addr[pipe_ndx] = None;
                self.txs[thread_ndx].send(ThreadTxMessage::UnsetWeights(local_pipe_ndx)).unwrap();
            },
        }
    }

    pub fn get_addr_pipe_ndx(&self, addr: u32) -> Option<usize> {
        match self.addr_to_pipe.get(&addr) {
            None => None,
            Some((thread_ndx, pipe_ndx)) => {
                Some(thread_ndx * self.pipe_count + pipe_ndx)
            },
        }
    }

    /// Finds an unused pipe and sets its theta and assigns it to the specified `addr` or updates an existing.
    pub fn set_addr_to_theta(
        &mut self,
        addr: u32,
        thetas: Vec<f32>,
    ) -> bool {
        match self.addr_to_pipe.get(&addr) {
            Some((thread_ndx, pipe_ndx)) => {
                self.txs[*thread_ndx].send(ThreadTxMessage::SetWeights(*pipe_ndx, thetas)).unwrap();
                true
            },
            None => {
                for x in 0..self.pipe_to_addr.len() {
                    if self.pipe_to_addr[x].is_none() {
                        self.pipe_to_addr[x] = Some(addr);
                        let thread_ndx = x / self.pipe_count;
                        let pipe_ndx = x - thread_ndx * self.pipe_count;
                        self.addr_to_pipe.insert(addr, (thread_ndx, pipe_ndx));
                        self.txs[thread_ndx].send(ThreadTxMessage::SetWeights(pipe_ndx, thetas)).unwrap();
                        return true;
                    }
                }
                false
            },
        }
    }

    /// Sends a buffer to all threads to be processed.
    pub fn send_buffer_to_all(&self, buffer: &Vec<u8>, streams: usize) {
        for tx in &self.txs {
            tx.send(ThreadTxMessage::Buffer(buffer.clone(), streams)).unwrap();
        }
    }
    
    /// Used when this structure is first created.
    pub fn push_tx(&mut self, sender: Sender<ThreadTxMessage>) {
        self.txs.push(sender);
    }
}

/// A collection of messages each thread understands.
pub enum ThreadTxMessage {
    /// Used to send a buffer to be processed to a thread.
    ///
    /// The first argument is the buffer. The second is the number
    /// of streams contained in the buffer.
    Buffer(Vec<u8>, usize),
    /// Used to set a theta to a constant value for a single pipe.
    SetWeights(usize, Vec<f32>),
    /// Used to revert a pipe back to a value that is randomly choosen per buffer process operation.
    UnsetWeights(usize),
}