use numtoa::NumToA;

pub struct StaticString {
    buffer: [u8; 1024],
    cursor: usize,
    num_lines: u8,
}

impl StaticString {
    pub fn new() -> Self {
        StaticString {
            buffer: [0u8; 1024],
            cursor: 0,
            num_lines: 1,
        }
    }

    pub fn push_bytes(&mut self, bytes: &[u8]) {
        self.buffer[self.cursor..self.cursor + bytes.len()].copy_from_slice(bytes);
        self.cursor += bytes.len();
    }

    pub fn push_str(&mut self, string: &str) {
        self.push_bytes(string.as_bytes())
    }

    pub fn push_u32(&mut self, val: u32) {
        let mut scratch = [0u8; 10];
        self.push_str(val.numtoa_str(10, &mut scratch));
    }

    pub fn push_u16(&mut self, val: u16) {
        let mut scratch = [0u8; 10];
        self.push_str(val.numtoa_str(10, &mut scratch));
    }

    pub fn push_space(&mut self) {
        self.push_str(&" ");
    }

    pub fn newline(&mut self) {
        self.num_lines += 1;
        self.push_str(&"\n");
    }

    pub fn num_lines(&self) -> u32 {
        self.num_lines as u32
    }

    pub fn get(&self) -> &str {
        unsafe { core::str::from_utf8_unchecked(&self.buffer[0..self.cursor]) }
    }
}
