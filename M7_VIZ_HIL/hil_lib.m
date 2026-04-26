%% hil_lib.m  —  HIL Communication Library  (Phase 10)
%
% PURPOSE:
%   Implements the MATLAB-side UART communication layer for HIL with ESP32.
%   Entirely isolated in M7 — no other module is aware this exists.
%   Gated by HIL_ENABLED: when false, all functions are no-ops.
%
% PROTOCOL (MATLAB → ESP32):
%   Frame: [0xAB] [0xCD] [PWM1_H] [PWM1_L] [PWM2_H] [PWM2_L]
%                        [PWM3_H] [PWM3_L] [CRC8]
%   = 9 bytes at 50 Hz = 450 bytes/s (well within 115200 baud capacity)
%
%   PWM channels:
%     PWM1: servo1 — rudder (delta_r)
%     PWM2: servo2 — stern plane (delta_s)
%     PWM3: ESC    — thruster (n_rpm)
%   Range: [1000, 2000] µs → stored as uint16 big-endian
%
% PROTOCOL (ESP32 → MATLAB, optional telemetry):
%   Frame: [0xDC] [0xBA] [STATUS] [ECHO1_H] [ECHO1_L]
%                        [ECHO2_H] [ECHO2_L] [ECHO3_H] [ECHO3_L] [CRC8]
%   = 10 bytes
%   STATUS byte bits:
%     bit 0: armed (1 = ESC armed and ready)
%     bit 1: servo1_active
%     bit 2: servo2_active
%     bit 3: fault (any actuator fault)
%
% CRC: CRC-8/SMBUS, polynomial 0x07
%
% FUNCTIONS:
%   hil_init         — open serial port, return hil struct
%   hil_send         — encode and transmit one PWM frame
%   hil_receive      — read and decode ESP32 telemetry (non-blocking)
%   hil_close        — flush and close serial port
%   hil_encode_frame — encode PWM → 9-byte packet (testable without hardware)
%   hil_decode_frame — decode 10-byte telemetry packet
%   hil_crc8         — compute CRC-8/SMBUS checksum
%   hil_validate_pwm — check PWM values are in [1000, 2000] range
%
% ESP32 FIRMWARE NOTE:
%   See hil_esp32_firmware.ino for the matching Arduino sketch.
%   The ESP32 must be flashed before HIL testing.
%
% AUTHOR: AUV Simulation Project — Phase 10

% =========================================================================
function hil = hil_init(port, baud, enabled)
%% hil_init  —  Open serial port and return HIL state struct
%
% INPUTS:
%   port      string  serial port name, e.g. 'COM3' (Windows) or '/dev/ttyUSB0' (Linux)
%   baud      integer baud rate (default: 115200)
%   enabled   logical true = real hardware, false = simulation-only (no-op)
%
% OUTPUT:
%   hil       struct  HIL state: serial object, counters, timing

if nargin < 2, baud    = 115200; end
if nargin < 3, enabled = false;  end

hil.enabled     = enabled;
hil.port        = port;
hil.baud        = baud;
hil.tx_count    = 0;      % frames transmitted
hil.rx_count    = 0;      % frames received
hil.err_count   = 0;      % CRC errors
hil.last_status = 0;      % last ESP32 status byte
hil.last_echo   = [1500; 1500; 1500];   % last echoed PWM
hil.armed       = false;

if ~enabled
    hil.serial = [];
    fprintf('HIL: disabled (simulation-only mode).\n');
    return
end

% Open serial port
try
    s = serialport(port, baud);
    configureTerminator(s, 'CR/LF');
    s.Timeout = 0.01;    % 10ms non-blocking read timeout
    flush(s);            % clear any stale data in buffer
    hil.serial = s;
    fprintf('HIL: opened %s at %d baud.\n', port, baud);
catch ME
    warning('HIL: failed to open %s — %s\nRunning in disabled mode.', port, ME.message);
    hil.serial  = [];
    hil.enabled = false;
end

end

% =========================================================================
function hil = hil_send(hil, pwm)
%% hil_send  —  Encode and transmit one PWM frame to ESP32
%
% INPUTS:
%   hil   HIL state struct
%   pwm   [3×1] PWM values in microseconds [servo1; servo2; ESC]
%               Must be in [1000, 2000] range

if ~hil.enabled, return, end

% Validate and clamp PWM (safety — never send out-of-range to hardware)
pwm = hil_validate_pwm(pwm);

% Encode to 9-byte frame
frame = hil_encode_frame(pwm);

% Transmit
try
    write(hil.serial, frame, 'uint8');
    hil.tx_count = hil.tx_count + 1;
catch ME
    warning('HIL: transmit error — %s', ME.message);
    hil.err_count = hil.err_count + 1;
end

end

% =========================================================================
function [hil, status, echo_pwm] = hil_receive(hil)
%% hil_receive  —  Read and decode ESP32 telemetry (non-blocking)
%
% Returns immediately if no data available.
% Call at 50 Hz alongside hil_send.
%
% OUTPUTS:
%   hil       updated HIL state
%   status    uint8  ESP32 status byte (0 if no data)
%   echo_pwm  [3×1]  echoed PWM values from ESP32 (last known if no data)

status   = hil.last_status;
echo_pwm = hil.last_echo;

if ~hil.enabled, return, end
if isempty(hil.serial), return, end

try
    % Non-blocking check: read only if bytes available
    n_avail = hil.serial.NumBytesAvailable;
    if n_avail < 10
        return   % not enough bytes for a full telemetry frame
    end

    raw = read(hil.serial, 10, 'uint8');

    % Validate sync bytes
    if raw(1) ~= 0xDC || raw(2) ~= 0xBA
        hil.err_count = hil.err_count + 1;
        flush(hil.serial);   % discard corrupted frame
        return
    end

    % Validate CRC
    crc_received = raw(10);
    crc_computed = hil_crc8(raw(1:9));
    if crc_received ~= crc_computed
        hil.err_count = hil.err_count + 1;
        return
    end

    % Decode
    status   = raw(3);
    echo_pwm = [double(bitor(bitshift(uint16(raw(4)),8), uint16(raw(5))));
                double(bitor(bitshift(uint16(raw(6)),8), uint16(raw(7))));
                double(bitor(bitshift(uint16(raw(8)),8), uint16(raw(9))))];

    hil.last_status = status;
    hil.last_echo   = echo_pwm;
    hil.armed       = bitand(status, 1) > 0;
    hil.rx_count    = hil.rx_count + 1;

catch
    % Non-blocking — silently ignore read errors
end

end

% =========================================================================
function hil_close(hil)
%% hil_close  —  Flush and close serial port safely

if ~hil.enabled || isempty(hil.serial)
    return
end

% Send neutral PWM before closing — safe actuator state
safe_pwm = [1500; 1500; 1000];   % neutral fins, ESC disarmed
hil_send(hil, safe_pwm);
pause(0.05);

flush(hil.serial);
delete(hil.serial);
fprintf('HIL: port closed. TX=%d frames, RX=%d frames, Errors=%d\n', ...
    hil.tx_count, hil.rx_count, hil.err_count);

end

% =========================================================================
function frame = hil_encode_frame(pwm)
%% hil_encode_frame  —  Encode PWM values to 9-byte transmission frame
%
% FULLY TESTABLE WITHOUT HARDWARE.
%
% Frame layout:
%   [0xAB] [0xCD] [P1H] [P1L] [P2H] [P2L] [P3H] [P3L] [CRC8]
%
% All PWM values encoded as uint16 big-endian.

p1 = uint16(round(pwm(1)));
p2 = uint16(round(pwm(2)));
p3 = uint16(round(pwm(3)));

frame = uint8([
    0xAB;                          % sync byte 1
    0xCD;                          % sync byte 2
    bitshift(p1, -8);              % PWM1 high byte
    bitand(p1, 0xFF);              % PWM1 low byte
    bitshift(p2, -8);              % PWM2 high byte
    bitand(p2, 0xFF);              % PWM2 low byte
    bitshift(p3, -8);              % PWM3 high byte
    bitand(p3, 0xFF);              % PWM3 low byte
    0x00                           % CRC placeholder
]);

frame(9) = hil_crc8(frame(1:8));   % fill in CRC over bytes 1-8

end

% =========================================================================
function [status, echo_pwm, valid] = hil_decode_frame(raw)
%% hil_decode_frame  —  Decode 10-byte ESP32 telemetry frame
%
% FULLY TESTABLE WITHOUT HARDWARE.
%
% INPUTS:
%   raw   [10×1] uint8 raw bytes from ESP32
%
% OUTPUTS:
%   status    uint8  ESP32 status byte
%   echo_pwm  [3×1]  echoed PWM values
%   valid     logical true if sync and CRC are correct

status   = uint8(0);
echo_pwm = [1500; 1500; 1500];
valid    = false;

if numel(raw) < 10, return, end
raw = uint8(raw(:));

% Sync check
if raw(1) ~= 0xDC || raw(2) ~= 0xBA, return, end

% CRC check
if hil_crc8(raw(1:9)) ~= raw(10), return, end

status = raw(3);
echo_pwm = [
    double(bitor(bitshift(uint16(raw(4)), 8), uint16(raw(5))));
    double(bitor(bitshift(uint16(raw(6)), 8), uint16(raw(7))));
    double(bitor(bitshift(uint16(raw(8)), 8), uint16(raw(9))))
];
valid = true;

end

% =========================================================================
function crc = hil_crc8(data)
%% hil_crc8  —  CRC-8/SMBUS checksum (polynomial 0x07)
%
% Standard CRC-8 used in automotive and sensor protocols.
% Matches the Arduino CRC8 library used in the ESP32 firmware.

crc  = uint8(0);
poly = uint8(0x07);

for i = 1:numel(data)
    crc = bitxor(crc, uint8(data(i)));
    for bit = 1:8
        if bitand(crc, uint8(0x80)) > 0
            crc = bitxor(bitshift(crc, 1), poly);
        else
            crc = bitshift(crc, 1);
        end
    end
end

end

% =========================================================================
function pwm_safe = hil_validate_pwm(pwm)
%% hil_validate_pwm  —  Safety clamp PWM to [1000, 2000] µs
%
% NEVER sends out-of-range values to hardware.
% Out-of-range would command actuators beyond mechanical limits.

pwm_safe = max(1000, min(2000, double(pwm(:))));

% Warn if clamping occurred — indicates upstream calculation error
if any(abs(pwm_safe - pwm(:)) > 0.5)
    warning('hil_validate_pwm: PWM clamped. Check actuation module output.');
end

end

% =========================================================================
function hil_print_status(hil)
%% hil_print_status  —  Print HIL session statistics

fprintf('\n=== HIL Session Status ===\n');
fprintf('  Port:         %s @ %d baud\n', hil.port, hil.baud);
fprintf('  Enabled:      %d\n', hil.enabled);
fprintf('  TX frames:    %d\n', hil.tx_count);
fprintf('  RX frames:    %d\n', hil.rx_count);
fprintf('  CRC errors:   %d\n', hil.err_count);
fprintf('  Armed:        %d\n', hil.armed);
fprintf('  Last echo:    [%d, %d, %d] µs\n', ...
    hil.last_echo(1), hil.last_echo(2), hil.last_echo(3));
fprintf('==========================\n\n');

end
