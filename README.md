# LoRa multihop communication using RPL

## Overview

This repository contains a modified version of Contiki OS with support for the MSP-EXP430FR5994 LaunchPad and the SX1276 LoRa radio transceiver. The project enables LoRa-based multihop communication using the RPL routing protocol, allowing data collection over large agricultural areas. Example applications can be found in:
- `examples/LoRa-transmitter`
- `examples/LoRa-receiver`
- `examples/LoRa-tests`

## Problem

Precision agriculture applications involve collecting environmental data from large areas using wireless sensor networks. These networks must operate under strict constraints regarding transmission range, packet delivery rate, latency, and battery life. LoRa technology can provide long-range communication for low-power devices, but most deployments rely on single-hop architectures. This project explores the use of the RPL in order to enable multihop communication in LoRa sensor networks, extending coverage and improving scalability.

## Repository Structure

- `platform/expfr5994` – platform support for the MSP-EXP430FR5994 LaunchPad 
- `platform/expfr5994/radio` – implementation of the SX1276 LoRa transceiver  
- `examples/LoRa-tests` – code used for field evaluation, tests 5-7 enable LoRa multihop communication with RPL

## Compiling the Code for MSP-EXP430FR5994 with SX1276

1. Clone this repository
2. Navigate to one of the example directories:
   - `examples/LoRa-transmitter`
   - `examples/LoRa-receiver`
   - `examples/LoRa-tests`
3. Compile the firmware using `make TARGET=expfr5994`

## Platform

This project is implemented on top of Contiki (https://github.com/contiki-os/contiki), an open-source operating system for IoT devices.
