# meta-hackathon-aipool

The goal of this project is to efficiently automate repetitive pool maintenance, specifically backwashing the filter.

The plan is to implement the pool logic - read the pressure gauge from the filter, control the pump and the backwash valve and adjust all components to work together.

## Components

* qr code reader
* mDNS responder
* http server
* config manager
* pool logic controller
* home assistant integration

## Existing libs

* gauge
* relays
* pool pump

## Hardware

This hack requires building out physical hardware as described in the plans.
The hardware setup requires two power supplies, 5 and 12VDC and 24VAC, rs485 adapter, relay board, esp32cam, valve actuator, pump motor and wiring.
