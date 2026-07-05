# 24 - Diagnostics

## Contents

- [Overview](#overview)
- [Status separation](#status-separation)
- [Capability scanner](#capability-scanner)
- [CAN sniffer workflow](#can-sniffer-workflow)
- [Vehicle troubleshooting](#vehicle-troubleshooting)

## Overview

Diagnostics must answer one question quickly: what works, what does not, and where the failure sits.

## Status separation

| Status | Meaning |
| --- | --- |
| ESP-NOW | Valid packets are received from sender. |
| CAN | Sender CAN/TWAI is active and not faulted. |
| OBD | ECU replies to OBD requests. |
| UDS | UDS scan/request state. |
| Simulation | Runtime simulation enabled/disabled. |

## Capability scanner

The scanner should report:

- supported OBD PID masks,
- probed PID response status,
- UDS reachable ECUs,
- UDS DID status,
- DTC results,
- CAN sniffer candidates.

## CAN sniffer workflow

1. Start passive sniffer.
2. Capture baseline.
3. Trigger vehicle action.
4. Capture changed frames.
5. Show candidate identifiers, bytes and bits.
6. Export JSON.

## Vehicle troubleshooting

If no values arrive:

1. Check sender web status.
2. Download diagnostic log.
3. Check if OBD requests were sent.
4. Check if ECU responses arrived.
5. Check if telemetry sequence increments.
6. Check display packet age and packet loss.

