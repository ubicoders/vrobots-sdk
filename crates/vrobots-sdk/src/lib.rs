//! Rust client SDK for the Ubicoders virtual robots simulator.
//!
//! A safe wrapper over the VRobots SDK C ABI, as exposed by the
//! `vrobots-sdk-sys` crate. The same core library backs the Python and C++
//! front ends, so all three report the same behaviour.
//!
//! Not implemented yet: only the version constant is available so far.

/// The version of this crate, which matches the SDK release it wraps.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
