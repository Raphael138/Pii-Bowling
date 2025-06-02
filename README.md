# Pii-Bowling 🎳  
*A Wii Bowling Replica using Raspberry Pi Pico W boards*

[![Watch the Demo](https://img.youtube.com/vi/bTKnEO6jgwE/0.jpg)](https://www.youtube.com/watch?v=bTKnEO6jgwE)

## Overview

Pii-Bowling is a two-player bowling game that mimics the mechanics of Wii Bowling using Raspberry Pi Pico W microcontrollers. Built as a final project for [ECE 4760: Digital Systems Design Using Microcontrollers](https://ece4760.github.io/), the game combines embedded systems, wireless communication, and motion sensing to create an interactive bowling experience.

## Project Structure

This repository is organized into the following subdirectories:

- [`src/`](./src): Contains firmware for both the **remote** (bowling controller) and **console** (score display using VGA screen and physics computation) Raspberry Pi Pico W boards.
- [`website/`](./website): Contains the static site files for the project presentation website.

You can view the full project writeup here: [Pii-Bowling Project Webpage](https://ece4760.github.io/Projects/Spring2025/rft38_kjm264/index.html)

## Features

- **Motion-Controlled Gameplay**: Uses IMU data to detect a "throw" gesture and determine bowling ball velocity.
- **Wireless Communication**: Remote and console boards communicate via UDP over WiFi.
- **Physics Simulation**: Custom physics engine simulates ball trajectory and pin collisions in real-time.
- **TFT Display**: The console board renders the lane, ball, and pins on a 320x240 VGA display.
- **Scorekeeping**: Implements standard ten-pin bowling rules for scoring, including spares and strikes.

## Credits

Developed by **Raphael Thesmar** and **Kristen Moon** for Spring 2025 ECE 4760.