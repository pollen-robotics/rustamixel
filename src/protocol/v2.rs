use core::ops;
#[cfg(not(feature = "std"))]
use alloc::Vec;

use crc16;
use hal;

use error::{DynamixelError, ErrorType};
use motors::Register;

const TIMEOUT: hal::time::MilliSecond = hal::time::MilliSecond(1);

/// Dynamixel controller for the protocol v2
pub struct ControllerV2<RX, TX, CLOCK> {
    rx: RX,
    tx: TX,

    clock: CLOCK,
    timeout: hal::time::MilliSecond,
}

impl<RX, TX, CLOCK> ControllerV2<RX, TX, CLOCK>
where
    TX: hal::serial::Write<u8, Error = !>,
    RX: hal::serial::Read<u8, Error = !>,
    CLOCK: hal::time::Time,
{
    /// Create a new controller for the protocol v2.
    pub fn new(rx: RX, tx: TX, clock: CLOCK) -> ControllerV2<RX, TX, CLOCK> {
        ControllerV2 {
            rx,
            tx,
            clock,
            timeout: TIMEOUT,
        }
    }
    /// Send a ping signal to the specified motor
    pub fn ping(&mut self, id: u8) -> Result<bool, DynamixelError> {
        self.send(&InstructionPacket::ping(id));

        match self.recv() {
            Ok(_) => Ok(true),
            Err(e) => {
                if e.error == ErrorType::Timeout {
                    Ok(false)
                } else {
                    Err(e)
                }
            }
        }
    }
    /// Scan a range of motors id
    pub fn scan(&mut self, id_range: ops::Range<u8>) -> Result<Vec<u8>, DynamixelError> {
        let mut v = Vec::new();

        for id in id_range {
            if self.ping(id)? {
                v.push(id);
            }
        }

        Ok(v)
    }
    /// Read data from a specified register `REG` on motor `id`.
    ///
    /// *Note: This will send an InstructionPacket to the motor and block until the StatusPacket is received as reponse.*
    pub fn read_data<REG>(&mut self, id: u8, reg: &REG) -> Result<u16, DynamixelError>
    where
        REG: Register,
    {
        let packet = InstructionPacket::read_data(id, reg.address(), reg.length());

        self.send(&packet);
        let status = self.recv()?;

        if (status.parameters.len()) != reg.length() as usize {
            return Err(DynamixelError::parsing_error());
        }

        Ok(dxl_decode_data!(reg.length(), status.parameters))
    }
    /// Sync read data from a specified register `REG` on a list of motor `id`.
    ///
    /// *Note: This will send an InstructionPacket to all targeted motors and block until all the StatusPackets are received as reponse.*
    pub fn sync_read_data<REG>(&mut self, ids: &[u8], reg: &REG) -> Vec<(u8, u16)>
    where
        REG: Register,
    {
        let packet = InstructionPacket::sync_read_data(ids, reg.address(), reg.length());
        self.send(&packet);

        let mut answer = Vec::new();

        for &id in ids {
            if let Ok(status_packet) = self.recv() {
                answer.push((id, dxl_decode_data!(reg.length(), status_packet.parameters)));
            }
        }

        answer
    }
    /// Write `data` to a specified register `REG` on motor `id`.
    ///
    /// *Note: This will send an InstructionPacket to the motor and block until the StatusPacket is received as an acknowledgment.*
    pub fn write_data<REG>(&mut self, id: u8, reg: &REG, data: u16) -> Result<(), DynamixelError>
    where
        REG: Register,
    {
        let packet = InstructionPacket::write_data(id, reg.address(), reg.length(), data);

        self.send(&packet);
        self.recv()?;

        Ok(())
    }
    /// Sync write `data` to a specified register `REG` on a list of motor `ids`.
    ///
    /// *Note: The motors will not answer after a SyncWrite. `sync_write_data` only blocks during the sending.*
    pub fn sync_write_data<REG>(&mut self, reg: &REG, data: &[(u8, u16)])
    where
        REG: Register,
    {
        let packet = InstructionPacket::sync_write_data(reg.address(), reg.length(), data);

        self.send(&packet);
    }

    fn send(&mut self, packet: &InstructionPacket) {
        for b in packet.as_bytes() {
            block!(self.tx.write(b)).ok();
        }
    }
    fn recv(&mut self) -> Result<StatusPacket, DynamixelError> {
        let mut bytes = Vec::new();
        for _ in 0..PacketHeader::length() {
            bytes.push(busy_wait!(self.rx.read(), self.clock, self.timeout)?);
        }
        let header = PacketHeader::from_bytes(&bytes)?;

        for _ in 0..header.length {
            bytes.push(busy_wait!(self.rx.read(), self.clock, self.timeout)?);
        }

        let p = StatusPacket::from_bytes(&bytes)?;

        if let Some(e) = p.error_code {
            return Err(DynamixelError::status_error_code(e));
        }

        Ok(p)
    }
}

#[derive(Clone, Copy, Debug)]
enum Instruction {
    Ping = 0x01,
    ReadData = 0x02,
    WriteData = 0x03,
    _Reset = 0x06,
    SyncRead = 0x82,
    SyncWrite = 0x83,
}

const BROADCAST_ID: u8 = 254;

/// Packet header are constructed as follows [0xFF, 0xFF, 0xFD, 0x00, ID, `LEN_L`, `LEN_H`]
#[derive(Debug)]
struct PacketHeader {
    _id: u8,
    length: u16,
}
impl PacketHeader {
    fn from_bytes(bytes: &[u8]) -> Result<PacketHeader, DynamixelError> {
        const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];

        assert_eq!(bytes.len(), PacketHeader::length());

        if bytes[..4] != HEADER {
            return Err(DynamixelError::parsing_error());
        }

        Ok(PacketHeader {
            _id: bytes[4],
            length: pack!(bytes[5], bytes[6]),
        })
    }
    const fn length() -> usize {
        7
    }
}

#[derive(Debug)]
struct InstructionPacket {
    id: u8,
    length: u16,
    instruction: Instruction,
    parameters: Vec<u8>,
}
impl InstructionPacket {
    fn new(id: u8, instruction: Instruction, parameters: Vec<u8>) -> InstructionPacket {
        InstructionPacket {
            id,
            length: (parameters.len() + 3) as u16,
            instruction,
            parameters,
        }
    }
    fn ping(id: u8) -> InstructionPacket {
        InstructionPacket::new(id, Instruction::Ping, vec![])
    }
    fn read_data(id: u8, addr: u16, len: u16) -> InstructionPacket {
        let (addr_l, addr_h) = unpack!(addr);
        let (len_l, len_h) = unpack!(len);

        InstructionPacket::new(
            id,
            Instruction::ReadData,
            vec![addr_l, addr_h, len_l, len_h],
        )
    }
    fn sync_read_data(ids: &[u8], addr: u16, len: u16) -> InstructionPacket {
        let (addr_l, addr_h) = unpack!(addr);
        let (len_l, len_h) = unpack!(len);

        let mut param = vec![addr_l, addr_h, len_l, len_h];
        param.extend(ids);

        InstructionPacket::new(BROADCAST_ID, Instruction::SyncRead, param)
    }
    fn write_data(id: u8, addr: u16, len: u16, data: u16) -> InstructionPacket {
        let (addr_l, addr_h) = unpack!(addr);

        let mut parameters = vec![addr_l, addr_h];
        parameters.extend(dxl_code_data!(len, data));
        InstructionPacket::new(id, Instruction::WriteData, parameters)
    }
    fn sync_write_data(addr: u16, len: u16, data: &[(u8, u16)]) -> InstructionPacket {
        let (addr_l, addr_h) = unpack!(addr);
        let mut param = vec![addr_l, addr_h];

        let coded_data = data.iter().fold(Vec::new(), |mut acc, &(id, data)| {
            acc.push(id);
            acc.extend(dxl_code_data!(len, data));
            acc
        });

        param.extend(coded_data);

        InstructionPacket::new(BROADCAST_ID, Instruction::SyncWrite, param)
    }
    /// [0xFF, 0xFF, 0xFD, 0x00, ID, LEN_L, LEN_H, INST, PARAM 1, PARAM 2, ..., PARAM N, CRC_L, CRC_H]
    fn as_bytes(&self) -> Vec<u8> {
        let (len_l, len_h) = unpack!(self.length);

        let mut buff = vec![
            0xFF,
            0xFF,
            0xFD,
            0x00,
            self.id,
            len_l,
            len_h,
            self.instruction as u8,
        ];

        buff.extend(&self.parameters);

        let (crc_l, crc_h) = unpack!(crc(&buff));
        buff.push(crc_l);
        buff.push(crc_h);

        buff
    }
}

/// Status Packet are constructed as follows:
/// [0xFF, 0xFF, 0xFD, 0x00, ID, `LEN_L`, `LEN_H`, 0x55, ERROR, PARAM 1, PARAM 2, ..., PARAM N, `CRC_L`, `CRC_H`]
#[derive(Debug)]
struct StatusPacket {
    _id: u8,
    _length: u16,
    error_code: Option<u8>,
    parameters: Vec<u8>,
}
impl StatusPacket {
    fn from_bytes(bytes: &[u8]) -> Result<StatusPacket, DynamixelError> {
        let end = bytes.len();
        if crc(&bytes[..end - 2]) != pack!(bytes[end - 2], bytes[end - 1]) {
            return Err(DynamixelError::invalid_checksum());
        }

        let _id = bytes[4];
        let _length = pack!(bytes[5], bytes[6]);
        let error_code = if bytes[8] == 0 { None } else { Some(bytes[8]) };
        let parameters = bytes[9..end - 2].to_vec();
        Ok(StatusPacket {
            _id,
            _length,
            error_code,
            parameters,
        })
    }
    #[cfg(test)]
    /// [0xFF, 0xFF, 0xFD, 0x00, ID, `LEN_L`, `LEN_H`, 0x55, ERROR, PARAM 1, PARAM 2, ..., PARAM N, `CRC_L`, `CRC_H`]
    fn to_bytes(&self) -> Vec<u8> {
        let (len_l, len_h) = unpack!(self._length);
        let mut bytes = vec![
            0xFF,
            0xFF,
            0xFD,
            0x00,
            self._id,
            len_l,
            len_h,
            0x55,
            self.error_code.unwrap_or(0),
        ];
        bytes.extend(&self.parameters);
        let crc = crc(&bytes);
        let (crc_l, crc_h) = unpack!(crc);
        bytes.extend(vec![crc_l, crc_h]);
        bytes
    }
}

fn crc(bytes: &[u8]) -> u16 {
    crc16::State::<crc16::BUYPASS>::calculate(bytes)
}

#[cfg(test)]
mod test {
    extern crate rand;

    // use nb;
    use super::*;
    use self::rand::random;
    use self::rand::distributions::{Range, Sample};

    #[test]
    fn parse_status_packet() {
        let bytes = [0xFF, 0xFF, 0xFD, 0x00, 42, 6, 0, 0x55, 0, 0, 23, 4, 242];
        let sp = StatusPacket::from_bytes(&bytes).unwrap();
        assert_eq!(sp._id, 42, "check id");
        assert_eq!(sp._length, pack!(6_u8, 0_u8), "check length");
        assert_eq!(sp.parameters, vec![0, 23], "check parameters");
    }
    #[test]
    fn parse_random_status_packet() {
        let mut rp = random_status_packet();
        rp.error_code = None;
        let bytes = rp.to_bytes();

        let sp = StatusPacket::from_bytes(&bytes).unwrap();
        assert_eq!(sp._id, rp._id, "check id");
        assert_eq!(sp._length, rp._length, "check length");
        assert!(sp.error_code.is_none(), "check error code");
        assert_eq!(sp.parameters, rp.parameters, "check parameters");
    }
    #[test]
    fn status_error() {
        let error: u8 = random();
        let error = if error == 0 { 1 } else { error };
        let mut bytes = vec![0xFF, 0xFF, 0xFD, 0x00, 42, 6, 0, 0x55, error, 0, 23];
        let crc = crc(&bytes);
        let (crc_l, crc_h) = unpack!(crc);
        bytes.extend(vec![crc_l, crc_h]);
        let sp = StatusPacket::from_bytes(&bytes).unwrap();

        assert_eq!(sp.error_code, Some(error));
    }
    fn random_status_packet() -> StatusPacket {
        let _id: u8 = random();
        let parameters = random_parameters();
        let _length = (parameters.len() + 4) as u16;
        let error_code = random_error();
        StatusPacket {
            _id,
            _length,
            error_code,
            parameters,
        }
    }
    fn random_error() -> Option<u8> {
        let e = Range::new(0u8, 8).sample(&mut rand::thread_rng());
        match e {
            0 => None,
            e => Some(e),
        }
    }
    fn random_parameters() -> Vec<u8> {
        let size: u8 = random();
        let mut data = Vec::new();
        for _ in 0..size {
            data.push(random());
        }
        data
    }
    // struct FakeRx;
    // impl FakeRx {}
    // impl hal::serial::Read<u8> for FakeRx {
    //     type Error = !;
    //     fn read(&mut self) -> nb::Result<u8, Self::Error> {
    //         Ok(self.read())
    //     }
    // }
    // struct FakeTx;
    // impl hal::serial::Write<u8> for FakeTx {
    //     type Error = !;
    //     fn write(&mut self, _: u8) -> nb::Result<(), Self::Error> {
    //         Ok(())
    //     }
    //     fn flush(&mut self) -> nb::Result<(), Self::Error> {
    //         Ok(())
    //     }
    //     fn complete(&self) -> nb::Result<(), Self::Error> {
    //         Ok(())
    //     }
    // }
}
