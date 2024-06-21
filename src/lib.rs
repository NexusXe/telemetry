#![cfg_attr(not(test), no_std)]
#![feature(const_trait_impl)]
#![feature(ascii_char)]

#[derive(Debug)]
pub enum NmeaParseError {
    InvalidInputError(u8),
    ImpossibleMessageTypeError([u8; 3]),
    SectionParseError,
    MessageTooLongError(usize),
    IllegalCharacterError(u8),
    MalformedPrefixError([u8; 8]),
    MalformedSuffixError([u8; 8]),
    ChecksumError { expected: u8, actual: u8 },
}

pub enum NmeaMessageType {
    GGA,
    GLL,
    GSA,
    GSV,
    RMV,
    VTG,
    TXT,
}

pub struct RawNmeaMessage {
    message: [u8; Self::NMEA_BUFFER_SIZE],
    length: usize,
}

impl RawNmeaMessage {
    pub fn new<const STRICT: bool>(input: &[u8]) -> Result<Self, NmeaParseError> { // should STRICT be a generic parameter?
        fn first_non_ascii_byte(slice: &[u8]) -> u8 {
            for &byte in slice {
                if byte >= 128 {
                    return byte;
                }
            }
            unreachable!()
        }

        if !input.is_ascii() {
            return Err(NmeaParseError::InvalidInputError(first_non_ascii_byte(
                input,
            )));
        } // is the input all valid ASCII?

        if input.len() > Self::NMEA_BUFFER_SIZE {
            // is the input a reasonable (valid) size?
            return Err(NmeaParseError::MessageTooLongError(input.len()));
        }

        // does the message start as we would expect?
        const PREFIX_MASK: u64 = u64::from_be_bytes(*b"$GP\0\0\0,\0");
        const FIXED_PREFIX_CHARS: u64 = 0xFFFFFF000000FF00;
        let mut prefix_bytes: [u8; 8] = [0u8; 8];
        prefix_bytes.copy_from_slice(&input[..8]);
        let prefix_check: u64 =
            (u64::from_be_bytes(prefix_bytes) ^ PREFIX_MASK) & FIXED_PREFIX_CHARS;
        if prefix_check != 0 {
            return Err(NmeaParseError::MalformedPrefixError(
                prefix_check.to_be_bytes(),
            ));
        }

        // does the message end as we would expect?
        const SUFFIX_MASK: u64 = u64::from_be_bytes(*b"\0\0\0*\0\0\r\n");
        const FIXED_SUFFIX_CHARS: u64 = 0x000000FF0000FFFF;
        let mut suffix_bytes: [u8; 8] = [0u8; 8];
        suffix_bytes.copy_from_slice(&input[(input.len() - 4)..]);
        let suffix_check: u64 =
            (u64::from_be_bytes(suffix_bytes) ^ SUFFIX_MASK) & FIXED_SUFFIX_CHARS;
        if suffix_check != 0 {
            return Err(NmeaParseError::MalformedSuffixError(
                suffix_check.to_be_bytes(),
            ));
        }

        // construct potential output struct (early so that we can verify its checksum)
        let mut outbuf = [0u8; Self::NMEA_BUFFER_SIZE];
        outbuf.copy_from_slice(input);
        let output: Self = Self {
            message: outbuf,
            length: input.len(),
        };

        let checksum_is_valid: bool = output.verify_checksum()?;
        if STRICT && !checksum_is_valid {
            return Err(NmeaParseError::ChecksumError {
                expected: output.get_checksum()?,
                actual: output.calculate_checksum(),
            });
        }

        Ok(output)
    }
    const NMEA_BUFFER_SIZE: usize = 92;
    pub const fn r#type(&self) -> Result<NmeaMessageType, NmeaParseError> {
        let message_type_bytes: [u8; 3] = [self.message[3], self.message[4], self.message[5]];

        match &message_type_bytes {
            b"GGA" => Ok(NmeaMessageType::GGA), // time, lat, long, fix type, number of sats, HDOP, _, altitude, _, _, _, checksum
            b"GLL" => Ok(NmeaMessageType::GLL), // lat, ldir, long, ldir, time, status, mode indicator, checksum
            b"GSA" => Ok(NmeaMessageType::GSA),
            b"GSV" => Ok(NmeaMessageType::GSV),
            b"RMV" => Ok(NmeaMessageType::RMV),
            b"VTG" => Ok(NmeaMessageType::VTG),
            b"TXT" => Ok(NmeaMessageType::TXT),
            _ => Err(NmeaParseError::ImpossibleMessageTypeError(
                message_type_bytes,
            )),
        }
    }

    #[allow(clippy::len_without_is_empty)]
    pub const fn len(&self) -> usize {
        self.length
    }

    fn conv_checksum(input: [u8; 2]) -> Result<u8, NmeaParseError> {
        const fn hex_byte_to_byte(c: u8) -> Result<u8, NmeaParseError> {
            match c {
                b'0'..=b'9' => Ok(c - b'0'),
                b'A'..=b'F' => Ok(c - b'A' + 10),
                _ => Err(NmeaParseError::InvalidInputError(c)),
            }
        }
        Ok((hex_byte_to_byte(input[0])? << 4) | (hex_byte_to_byte(input[1])?))
    }

    fn get_checksum(&self) -> Result<u8, NmeaParseError> {
        let checksum_bytes: [u8; 2] = [self.message[self.len() - 4], self.message[self.len() - 3]];
        Self::conv_checksum(checksum_bytes)
    }

    fn calculate_checksum(&self) -> u8 {
        self.message[1..self.len() - 5]
            .iter()
            .fold(0, |acc, &x| acc ^ x) // edge case optimization: init with first element
    }

    pub fn verify_checksum(&self) -> Result<bool, NmeaParseError> {
        Ok(self.get_checksum()? == self.calculate_checksum())
    }
}

pub trait SmallAxis {
    fn from_nmea(nmea: &str) -> Self;
}

pub struct SmallLat([u8; 3]);
impl const SmallAxis for SmallLat {
    fn from_nmea(nmea: &str) -> Self {
        todo!()
    }
}
pub struct SmallLong([u8; 3]);

pub struct CompactLocation {
    // With 3 bytes for latitude and 3 bytes for longitude
    lat: [u8; 3],
    lng: [u8; 3],
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_checksum_twiddling() {
        const INVALID: [u8; 2] = [b'A', b'X'];
        const INPUT: [u8; 2] = [b'A', b'9'];

        let rt_valid_csum: u8 = RawNmeaMessage::conv_checksum(INPUT).unwrap();
        assert_eq!(0xA9, rt_valid_csum);
        let rt_invalid_csum = RawNmeaMessage::conv_checksum(INVALID);
        assert!(rt_invalid_csum.is_err());
    }

    #[test]
    fn checksum_calculation() {
        let nmea1 = b"$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*43\r\n";
        let mut message = [0u8; RawNmeaMessage::NMEA_BUFFER_SIZE];
        message[..nmea1.len()].copy_from_slice(nmea1);
        let test_message: RawNmeaMessage = RawNmeaMessage {
            message,
            length: nmea1.len(),
        };

        println!(
            "{}",
            String::from_utf8(test_message.message[1..test_message.len() - 5].to_vec()).unwrap()
        );
        assert!(
            test_message.verify_checksum().is_ok(),
            "{:#?}",
            test_message.verify_checksum()
        );
        assert_eq!(
            test_message.get_checksum().unwrap(),
            test_message.calculate_checksum(),
            "rx: {:2X?} != calc: {:2X?}",
            [
                test_message.get_checksum().unwrap() >> 4,
                test_message.get_checksum().unwrap() & 0x0F
            ],
            [
                test_message.calculate_checksum() >> 4,
                test_message.calculate_checksum() & 0x0F
            ]
        );
        assert!(test_message.verify_checksum().unwrap())
    }
}
