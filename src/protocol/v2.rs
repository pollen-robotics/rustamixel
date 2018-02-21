use hal;
use crc16;

use error::DynamixelError;
use motors::Register;

pub struct V2<RX, TX> {
    _rx: RX,
    tx: TX,
}

impl<RX, TX> V2<RX, TX>
where
    TX: hal::serial::Write<u8, Error = !>,
{
    pub fn new(rx: RX, tx: TX) -> V2<RX, TX> {
        V2 { _rx: rx, tx }
    }
    fn send(&mut self, packet: &InstructionPacket) {
        for b in packet.as_bytes() {
            block!(self.tx.write(b)).ok();
        }
    }
    fn recv(&mut self) -> Result<StatusPacket, DynamixelError> {
        // TODO: read header
        // TODO: read rest of message with header.data
        // TODO: check error flag
        // TODO: impl. timeout
        let bytes = vec![1, 2, 3, 4];
        StatusPacket::from_bytes(&bytes)
    }
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

    pub fn write_data<REG>(&mut self, id: u8, reg: REG, data: u16) -> Result<(), DynamixelError>
    where
        REG: Register,
    {
        let packet = InstructionPacket::write_data(id, reg.address(), reg.length(), data);

        self.send(&packet);
        self.recv()?;
        Ok(())
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

        let (crc_l, crc_h) = unpack!(self.crc(&buff));
        buff.push(crc_l);
        buff.push(crc_h);

        buff
    }
    fn crc(&self, bytes: &[u8]) -> u16 {
        crc16::State::<crc16::BUYPASS>::calculate(bytes)
    }
}

struct StatusPacket {
    _id: u8,
    _error_code: Option<u8>,
    parameters: Vec<u8>,
}
impl StatusPacket {
    fn from_bytes(_bytes: &[u8]) -> Result<StatusPacket, DynamixelError> {
        Err(DynamixelError::parsing_error())
    }
}
