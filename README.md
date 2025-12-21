# Overview
## Multichannel Smart Tracking of REsistance for Adaptive Modulation (MultiSTREAM)
Continuously monitors the resistances of active channel(s) and adjusts voltage(s) to maintain target resistance(s) within tolerance

Multiple feedback sequences can be run

Active channels are configured by setting target resistances to a defined value
> Feedback mode activates as long as one of the target resistances is set to a defined value
> 
> Resistances of active channels will be monitored for feedback control

Monitor channels can be configured by setting the target resistance = None
> Voltages will be set to 0 V and be excluded from feedback control
>
> Continue being monitored and their readings logged

Channels can be configured for a voltage ramp down by setting the target resistance = -1 
> Rampdown mode activates if all target resistances are set to either None (monitor) or -1 (ramp down)

If any channel resistance > fail_resistance,
> voltages set to 0 V and channels will be permanently excluded from further feedback control, continue being monitored and their readings logged.

## Multichannel Demonstration
The target resistances of nine channels (red) are set in one sequence
> Eventually, all channels stabilised at their target resistances

A monitor channel (purple) is configured by setting its target resistance = None

<img src="images/9-heaters-644-925.svg" width="500" alt="Alt text">

# System Architecture
The control system consists of two main functions that work together:
## feedback_loop()
Monitors microheater resistances and adjusts voltages to stabilise microheaters at defined target resistances
## series_feedback()
Executes multiple feedback loops in a sequential order, with each feedback loop exiting after a specified hold_duration
> If feedback mode is active, hold_duration = None means the feedback loop will run indefinitely until KeyBoard Interrupt

# Feedback Mode
If the active channel(s) resistance more/less than target + tolerance:
the respective voltage(s) will decrease/increase by the voltage_step

Otherwise, the voltage(s) will be held constant

The system waits for update_time before next round of feedback control
# Rampdown Mode
If rampdown mode is active and a hold_duration is set, 
> Loop terminate after a specified duration when all channels finish ramping down to 0 V

# Batch Update Optimisation
For the 24-channel 48V-1A system, voltage updates are only sent to Teensy 4.1 for channels requiring adjustment

Teensy 4.1 communicates with DACs in batch mode rather than sequentially
> Minimises communication overhead and ensures synchronised control across all channels
# State Continuity Between Steps
Voltages: Last voltages from previous step become starting voltages for next step

CSV logging: all steps append to same CSV file

Real-time plots: plotting continues seamlessly across all steps

Failed channels: excluded for subsequent steps
