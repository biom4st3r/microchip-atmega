const std = @import("std");
const micro = @import("microzig");
const USART1 = micro.chip.types.peripherals.USART.USART1;

const unstable = micro.core.experimental.gpio;

pub const clock = struct {
    pub const Domain = enum {
        cpu,
    };
};

// All relavent addresses for referring to the port
// Derived from datasheet
fn Port(comptime io: u8) type {
    return struct {
        pub const io_port = io;
        pub const io_dir = io - 1;
        pub const io_pin = io - 2;

        pub const mem = @ptrFromInt(*volatile u8, io_pin + 0x20);
    };
}

// https://ww1.microchip.com/downloads/en/devicedoc/atmel-7766-8-bit-avr-atmega16u4-32u4_datasheet.pdf
// 31.0  Register Summary pg 414
const Ports = struct {
    pub const F = Port(0x11);
    pub const E = Port(0x0E);
    pub const D = Port(0x0B);
    pub const C = Port(0x08);
    pub const B = Port(0x05);
};

fn PortPin(comptime po: type, comptime pi: u3) type {
    return struct {
        pub const port = po;
        pub const pin = pi;
    };
}

pub fn parse_pin(comptime spec: []const u8) type {
    // Should be P[B-F][0-7] though some of those don't exist
    if (spec.len != 3) @compileError("Pin Name to long to exist: " ++ spec);
    const PORT = spec[1];
    const PIN = std.fmt.parseInt(u3, &.{spec[2]}, 10) catch unreachable;
    switch (PORT) {
        'F' => return PortPin(Ports.F, PIN),
        'E' => return PortPin(Ports.E, PIN),
        'D' => return PortPin(Ports.D, PIN),
        'C' => return PortPin(Ports.C, PIN),
        'B' => return PortPin(Ports.B, PIN),
        else => @compileError("Invalid Port" ++ spec),
    }
}

pub const gpio = struct {
    pub fn setOutput(comptime pp: type) void {
        micro.cpu.sbi(pp.port.io_dir, pp.pin);
    }
    pub fn setInput(comptime pp: type) void {
        micro.cpu.cbi(pp.port.io_dir, pp.pin);
    }
    pub fn read(comptime pp: type) unstable.State {
        return if (pp.port.mem.* & (1 << pp.pin) != 0) .high else .low;
    }
    pub fn write(comptime pp: type, state: unstable.State) void {
        if (state == .high) {
            // @compileLog(pp);
            micro.cpu.sbi(pp.port.io_port, pp.pin);
        } else {
            micro.cpu.cbi(pp.port.io_port, pp.pin);
        }
    }
    pub fn toggle(comptime pp: type) void {
        micro.cpu.sbi(pp.port.io_pin, pp.pin);
    }
};

// https://ww1.microchip.com/downloads/en/devicedoc/atmel-7766-8-bit-avr-atmega16u4-32u4_datasheet.pdf
// 18.0 USART pg 188
pub const uart = @import("ATmega328P.zig").uart;

// Copied from ATmega328P
pub fn Uart(comptime index: usize, comptime pins: micro.core.experimental.uart.Pins) type {
    if (index != 0) @compileError("Atmega328p only has a single uart!");
    if (pins.tx != null or pins.rx != null)
        @compileError("Atmega328p has fixed pins for uart!");

    return struct {
        const Self = @This();

        fn computeDivider(baud_rate: u32) !u12 {
            const pclk = micro.core.experimental.clock.get().cpu;
            const divider = ((pclk + (8 * baud_rate)) / (16 * baud_rate)) - 1;

            return std.math.cast(u12, divider) orelse return error.UnsupportedBaudRate;
        }

        fn computeBaudRate(divider: u12) u32 {
            return micro.core.experimental.clock.get().cpu / (16 * @as(u32, divider) + 1);
        }

        pub fn init(config: micro.core.experimental.uart.Config) !Self {
            const ucsz: u3 = switch (config.data_bits) {
                .five => 0b000,
                .six => 0b001,
                .seven => 0b010,
                .eight => 0b011,
                .nine => return error.UnsupportedWordSize, // 0b111
            };

            const upm: u2 = if (config.parity) |parity| switch (parity) {
                .even => @as(u2, 0b10), // even
                .odd => @as(u2, 0b11), // odd
            } else 0b00; // parity disabled

            const usbs: u1 = switch (config.stop_bits) {
                .one => 0b0,
                .two => 0b1,
            };

            const umsel: u2 = 0b00; // Asynchronous USART

            // baud is computed like this:
            //             f(osc)
            // BAUD = ----------------
            //        16 * (UBRRn + 1)

            const ubrr_val = try computeDivider(config.baud_rate);

            USART1.UCSR1A.modify(.{
                .MPCM0 = 0,
                .U2X0 = 0,
            });
            USART1.UCSR1B.write(.{
                .TXB80 = 0, // we don't care about these btw
                .RXB80 = 0, // we don't care about these btw
                .UCSZ02 = @truncate(u1, (ucsz & 0x04) >> 2),
                .TXEN0 = 1,
                .RXEN0 = 1,
                .UDRIE0 = 0, // no interrupts
                .TXCIE0 = 0, // no interrupts
                .RXCIE0 = 0, // no interrupts
            });
            USART1.UCSR1C.write(.{
                .UCPOL0 = 0, // async mode
                .UCSZ0 = @truncate(u2, (ucsz & 0x03) >> 0),
                .USBS0 = usbs,
                .UPM0 = upm,
                .UMSEL0 = umsel,
            });

            USART1.UBRR1.modify(ubrr_val);

            return Self{};
        }

        pub fn canWrite(self: Self) bool {
            _ = self;
            return (USART1.UCSR1A.read().UDRE0 == 1);
        }

        pub fn tx(self: Self, ch: u8) void {
            while (!self.canWrite()) {} // Wait for Previous transmission
            USART1.UDR1.* = ch; // Load the data to be transmitted
        }

        pub fn canRead(self: Self) bool {
            _ = self;
            return (USART1.UCSR1A.read().RXC0 == 1);
        }

        pub fn rx(self: Self) u8 {
            while (!self.canRead()) {} // Wait till the data is received
            return USART1.UDR1.*; // Read received data
        }
    };
}
