use crc16;
use hal;

use error::DynamixelError;
use motors::Register;

/// Dynamixel controller for the protocol v2
pub struct ControllerV2<RX, TX> {
    rx: RX,
    tx: TX,
}

impl<RX, TX> ControllerV2<RX, TX>
where
    TX: hal::serial::Write<u8, Error = !>,
    RX: hal::serial::Read<u8, Error = !>,
{
    /// Create a new controller for the protocol v2.
    pub fn new(rx: RX, tx: TX) -> ControllerV2<RX, TX> {
        ControllerV2 { rx: rx, tx }
    }
    /// Read data from a specified register `REG` on motor `id`.
    ///
    /// *Note: This will send an InstructionPacket to the motor and block until the StatusPacket is received as reponse.*
    pub fn read_data<REG>(&mut self, id: u8, reg: REG) -> Result<u16, DynamixelError>
    where
        REG: Register,
    {
        let packet = InstructionPacket::read_data(id, reg.address(), reg.length());

        self.send(&packet);
        let status = self.recv()?;

        if (status.parameters.len()) != reg.length() as usize {
            return Err(DynamixelError::parsing_error());
        }
        match reg.length() {
            1 => Ok(u16::from(status.parameters[0])),
            2 => Ok(pack!(status.parameters[0], status.parameters[1])),
            _ => Err(DynamixelError::unsupported_register()),
        }
    }
    /// Write `data` to a specified register `REG` on motor `id`.
    ///
    /// *Note: This will send an InstructionPacket to the motor and block until the StatusPacket is received as an acknowledgment.*
    pub fn write_data<REG>(&mut self, id: u8, reg: REG, data: u16) -> Result<(), DynamixelError>
    where
        REG: Register,
    {
        let packet = InstructionPacket::write_data(id, reg.address(), reg.length(), data);

        self.send(&packet);
        self.recv()?;

        Ok(())
    }
    fn send(&mut self, packet: &InstructionPacket) {
        for b in packet.as_bytes() {
            block!(self.tx.write(b)).ok();
        }
    }
    fn recv(&mut self) -> Result<StatusPacket, DynamixelError> {
        // TODO: impl. timeout

        let mut bytes = Vec::new();
        for _ in 0..PacketHeader::length() {
            bytes.push(block!(self.rx.read()).unwrap());
        }
        let header = PacketHeader::from_bytes(&bytes)?;

        for _ in 0..header.length {
            bytes.push(block!(self.rx.read()).unwrap());
        }

        let p = StatusPacket::from_bytes(&bytes)?;

        if let Some(e) = p.error_code {
            return Err(DynamixelError::status_error_code(e));
        }

        Ok(p)
    }
}

#[derive(Clone, Copy)]
enum Instruction {
    _Ping = 0x01,
    ReadData = 0x02,
    WriteData = 0x03,
    _Reset = 0x06,
    _SyncRead = 0x82,
    _SyncWrite = 0x83,
}

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
    fn read_data(id: u8, addr: u16, len: u16) -> InstructionPacket {
        let (addr_l, addr_h) = unpack!(addr);
        let (len_l, len_h) = unpack!(len);

        InstructionPacket::new(
            id,
            Instruction::ReadData,
            vec![addr_l, addr_h, len_l, len_h],
        )
    }
    fn write_data(id: u8, addr: u16, len: u16, data: u16) -> InstructionPacket {
        let (addr_l, addr_h) = unpack!(addr);
        let (len_l, len_h) = unpack!(len);

        let mut parameters = vec![addr_l, addr_h, len_l, len_h];
        match len {
            1 => parameters.push(data as u8),
            2 => {
                let (data_l, data_h) = unpack!(data);
                parameters.push(data_l);
                parameters.push(data_h);
            }
            _ => panic!("Unsupported data len"),
        }

        InstructionPacket::new(id, Instruction::WriteData, parameters)
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
}

fn crc(bytes: &[u8]) -> u16 {
    crc16::State::<crc16::BUYPASS>::calculate(bytes)
}

#[cfg(test)]
mod test {
    extern crate rand;

    use super::*;
    use self::rand::random;

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
        let id: u8 = random();
        let parameters = random_parameters();
        let len = (parameters.len() + 4) as u16;
        let (len_l, len_h) = unpack!(len);
        let error = 0;

        let mut bytes = vec![0xFF, 0xFF, 0xFD, 0x00, id, len_l, len_h, 0x55, error];
        bytes.extend(&parameters);
        let crc = crc(&bytes);
        let (crc_l, crc_h) = unpack!(crc);
        bytes.extend(vec![crc_l, crc_h]);

        let sp = StatusPacket::from_bytes(&bytes).unwrap();
        assert_eq!(sp._id, id, "check id");
        assert_eq!(sp._length, len, "check length");
        assert!(sp.error_code.is_none(), "check error code");
        assert_eq!(sp.parameters, parameters, "check parameters");
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
        println!("****{:?}", sp);

        assert_eq!(sp.error_code, Some(error));
    }
    fn random_parameters() -> Vec<u8> {
        let size: u8 = random();
        let mut data = Vec::new();
        for _ in 0..size {
            data.push(random());
        }
        data
    }
}
