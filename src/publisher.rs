//! UDP Publisher for ASTERIX data
//!
//! Simple wrapper around a UDP socket for transmitting ASTERIX blocks.

use anyhow::{Context, Result};
use std::net::UdpSocket;

/// UDP publisher for sending ASTERIX data blocks
pub struct Publisher {
    socket: UdpSocket,
    destination: String,
}

impl Publisher {
    /// Create a new publisher that sends to the specified destination
    pub fn new(destination: &str) -> Result<Self> {
        // Bind to any available local port
        let socket = UdpSocket::bind("0.0.0.0:0")
            .context("Failed to bind UDP socket")?;

        Ok(Self {
            socket,
            destination: destination.to_string(),
        })
    }

    /// Send an ASTERIX data block
    pub fn send(&self, data: &[u8]) -> Result<usize> {
        self.socket
            .send_to(data, &self.destination)
            .with_context(|| format!("Failed to send to {}", self.destination))
    }

    /// Get the local address the socket is bound to
    pub fn local_addr(&self) -> Result<std::net::SocketAddr> {
        self.socket
            .local_addr()
            .context("Failed to get local address")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_publisher_creation() {
        let publisher = Publisher::new("127.0.0.1:4000").unwrap();
        assert!(publisher.local_addr().is_ok());
    }
}
