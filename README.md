# Overview
## Multichannel Smart Tracking of REsistance for Adaptive Modulation (MultiSTREAM)
Provides automated voltage regulation for single channel or multichannels. <br>
<br>
Continuously monitors the resistances of active channel(s) and adjusts voltage(s) to maintain target resistance(s) within tolerance. <br>
<br>
Multiple feedback sequences can be run. <br> 
## Two-Channel Control Demonstration
The target resistances of two channels are gradually shifted apart in subsequent sequences. <br>
<br>
Eventually, one of the channel's resistances starts rising and can no longer meet its target resistance (highlighted in yellow). <br>
<br>
<img src="images/multisequence-control.svg" width="500" alt="Alt text">
# System Architecture
The control system consists of two main functions that work together:
## feedback_loop()
Monitors microheater resistances and adjusts voltages to stabilise microheaters at defined target resistances through continuous feedback control.
## series_feedback()
Executes multiple feedback loops in a sequential order, with each feedback loop exiting after a specified hold_duration. <br>
<br>
If hold_duration = None, the feedback loop runs indefinitely until KeyBoard Interrupt.
# Control Algorithm
For the 24-channel 48V-1A system, if any channel resistance > fail_resistance: <br>
voltage(s) will be set to 0 V and the channel(s) will be permanently excluded from further feedback control. <br>
<br>
Otherwise, the resistances of active channel(s) will be monitored for feedback control. <br>
<br>
If the active channel(s) resistance more/less than target + tolerance: <br>
the respective voltage(s) will decrease/increase by the voltage_step. <br>
<br>
Otherwise, the voltage(s) will be held constant. <br>
<br>
The system waits for update_time before next round of feedback control.
# Batch Update Optimisation
For the 24-channel 48V-1A system, voltage updates are only sent to Teensy 4.1 for channels requiring adjustment. <br>
<br>
Teensy 4.1 communicates with DACs in batch mode rather than sequentially to minimise communication overhead and ensure synchronised control across all channels.
# State Continuity Between Steps
Voltages: Last voltages from previous step become starting voltages for next step <br>
<br>
CSV logging: all steps append to same CSV file <br>
<br>
Real-time plots: plotting continues seamlessly across all steps <br>
<br>
Failed channels (implemented for 24-channel 48V-1A system): excluded for subsequent steps
