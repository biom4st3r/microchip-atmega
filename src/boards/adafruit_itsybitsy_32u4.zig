pub const clock_frequencies0 = .{
    .cpu = 16_000_000,
};

pub const clock_frequencies1 = .{
    .cpu = 8_000_000,
};

pub const clock_frequencies = clock_frequencies0;

pub const pin_map = .{
    .A0 = "PF7",
    .A1 = "PF6",
    .A2 = "PF5",
    .A3 = "PF4",
    .A4 = "PF1",
    .A5 = "PF0",

    .SCK = "PB1",
    .MOSI = "PB2",
    .MISO = "PB3",

    .D13 = "PC7",
    .D12 = "PD6",
    .D11 = "PB7",
    .D10 = "PB6",
    .D9 = "PB5",
    .D7 = "PE6",
    .D5 = "PC6",
    .SCL = "PD0",
    .SDA = "PD1",
    .D1 = "PD3",
    .D0 = "PD2",

    .D8 = "PB4",
    .D6 = "PD7",
    .D4 = "PD4",
};
