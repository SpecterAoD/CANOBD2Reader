# ISO-TP and OBD-II

`lib/isotp` implements the ISO 15765-2 receive path required for OBD-II replies
larger than one CAN frame.

## Supported frame types

- Single Frame
- First Frame
- Consecutive Frame
- Flow Control

The sender can receive responses from ECU IDs `0x7E8` through `0x7EF`.

## Reassembly behavior

`IsoTpReassembler` uses a static 4095-byte buffer and performs:

- First Frame length extraction
- Consecutive Frame sequence validation
- payload length tracking
- duplicate/overflow/invalid PCI rejection
- completion detection

No dynamic allocation is used.

## Flow Control

After a First Frame, the sender transmits a Flow Control CTS frame to the
responding ECU. Block size and STmin come from sender configuration.

## OBD integration

`OBDHandler` sends OBD requests through `IsoTpHandler` and accepts positive
responses for:

- Mode `0x01`: current live data PIDs
- Mode `0x02`: freeze frame data path support in the transport
- Mode `0x03`: DTC response transport
- Mode `0x09` PID `0x02`: VIN transport support

Negative responses (`0x7F`) are detected and rejected by the transport wrapper.

## Required hardware tests

Software tests cover frame assembly, CRC and decoder behavior. Vehicle-side
validation still requires hardware:

1. Single-frame Mode 01 PID response.
2. Multi-frame VIN via Mode 09 PID 02.
3. Mode 03 DTC response with and without active DTCs.
4. Timeout behavior when no ECU answers.
5. Multiple ECU responses in the `0x7E8..0x7EF` range.
