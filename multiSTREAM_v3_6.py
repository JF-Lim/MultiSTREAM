import numpy as np
import pandas as pd

import serial

import nidaqmx
from nidaqmx.constants import LineGrouping, AcquisitionType

import csv
from datetime import datetime
import matplotlib
matplotlib.use("TkAgg")  # Interactive backend for PyCharm/scripts
import matplotlib.pyplot as plt
from time import time, sleep

# Teensy 4.1 Voltage Command Setup
# Define and connect to Teensy
teensyport = "COM5" #COM-port that Teensy is using
teensy = serial.Serial(teensyport, 115200) #Serial("Port", Bautrate)
teensy.reset_input_buffer() # clear communication buffer containing strings sent from Teensy to Jupyter notebook
teensy.reset_output_buffer() # clear communication buffer containing strings sent from Jupyter notebook to Teensy
# NB: looks like these two functions don't always result in a cleared buffer!
# Alternatively the function "teensy.readlines()" could be used to clear the communication buffer from Teensy to jupyter notebook
teensy.timeout = 1 # max waittime for a string in s
teensy.write_timeout = 5 # max seconds for write action is 5s
print('Teensy connected')

def teensy_write(str):
    teensy.write((str+'\n').encode()) # send command with correct encoding and termination
    teensy.readline() # first line sent back is repetition of the command sent
    return

def teensy_readline():
    line = teensy.readline() # read line
    line = line.decode() # decode byte to string, i.e. get rid of starting 'b' character
    line = line.strip() # strip termination characters '\r\n'
    return line

def teensy_set_heaters(channel_voltage_pairs):
    """
    Update one or multiple heaters at once
    """
    # send all ChangeSetpoint commands first
    for channel_number, voltage in channel_voltage_pairs:
        command = "ChangeSetpoint " + str(channel_number) + " " + str(voltage)
        teensy_write(command)
        teensy_readline()  # Clear response

    # update setpoints and outputs at once
    teensy_write("UpdateSetpoints")
    teensy_readline()  # Clear response
    
    teensy_write("UpdateOutputs")
    teensy_readline()  # Clear response
    
    # Flush any remaining bytes
    teensy.reset_input_buffer()

    return

# NI 6002 DAQ Setup
# Mux switch delay = samples_per_switch / sample_rate
# For 1 or 2 channels, maximum is around 24,000 Hz sample rate and 400 samples per switch
# Trade off between multiplexer switch time vs standard deviation

sample_rate = 24000       # samples/s
samples_per_switch = 400  # no. of samples per mux channel

# open GLOBAL tasks that continue running
# digital inputs for P0.0-P0.4
dio_task = nidaqmx.Task()
dio_task.do_channels.add_do_chan(
    "Dev2/port0/line0:4",                             # we only need to change P0.0-P0.4
    line_grouping=LineGrouping.CHAN_FOR_ALL_LINES
)

# ai_task for voltage and current are defined in calculate_virp

def select_channel(channel_number):
    if not (1 <= channel_number <= 24):
        raise ValueError("Channel number must be between 1 and 24")
    
    # 5-bit address: [A4, A3, A2, A1, A0]
    bank = (channel_number - 1) // 6
    channel_in_bank = (channel_number - 1) % 6
    addr_bits = [int(b) for b in format(bank, '02b') + format(channel_in_bank, '03b')]  # [A4, A3, A2, A1, A0]

    # Reverse: P0.0 = A0, P0.1 = A1, ..., P0.4 = A4
    bits = addr_bits[::-1]

    # Convert to integer to write to P0.0–P0.4
    value = sum(bit << i for i, bit in enumerate(bits))
    dio_task.write(value, auto_start=True)

    #print(f"Channel {channel_number} selected:")
    #for i, val in enumerate(bits):
        #print(f"  P0.{i}: {'HIGH' if val else 'LOW'} → {3.3 if val else 0} V")
    
    return bits

def calculate_virp(channel_number):
    """
    Reads voltage and current for a specific channel, computes resistance and power.
    
    Args:
        channel_number (int): Channel number (1-24)
    
    Returns:
        v (float): voltage in V
        i (float): current in A
        r (float): resistance in Ohms
        p (float): power in W
    """
    # Select the channel
    select_channel(channel_number)
    
    # Wait 1 ms for multiplexer to settle
    sleep(0.001)
    
    # Start task, read samples, stop task
    with nidaqmx.Task() as ai_task:
        ai_task.ai_channels.add_ai_voltage_chan("Dev2/ai0")  # voltage
        ai_task.ai_channels.add_ai_voltage_chan("Dev2/ai1")  # current

        # finite sampling
        ai_task.timing.cfg_samp_clk_timing(
            rate=sample_rate,
            sample_mode=AcquisitionType.FINITE,
            samps_per_chan=samples_per_switch
        )

        data = ai_task.read(number_of_samples_per_channel=samples_per_switch)
    
    # Separate channel data
    voltage_samples = np.array(data[0])
    current_samples = np.array(data[1])
    
    # Compute mean values for stable readings
    v = 10 * np.mean(voltage_samples)      # hardware voltage divider
    i = np.mean(current_samples) / 0.5     # sense resistor = 0.5 Ohms
    r = v / i if i != 0 else float('inf')
    p = v * i
    
    return v, i, r, p

def initialise_channels(channel_voltage_pairs=None):
    """
    First, sets all 24 channels to 0V and prints VIRP values.
    Then, if channel_voltage_pairs is provided, sets selected channels to specified voltages,
    waits 1 second, and prints VIRP values for those selected channels.
    """
    # Set all 24 channels to 0V
    all_channels_zero = [(ch, 0) for ch in range(1, 25)]
    teensy_set_heaters(all_channels_zero)

    sleep(0.1)
    
    # Print VIRP for all channels
    print("All channels reset to 0V:")
    for channel in range(1, 25):
        v, i, r, p = calculate_virp(channel)
        print(f"Channel {channel:2d}: V={v:.4f} V, I={i:.4f} A, R={r:.4f} Ω, P={p:.4f} W")
    
    # If channel_voltage_pairs provided, set selected channels
    if channel_voltage_pairs is not None:
        print("\nSetting selected channels to specified voltages...")
        teensy_set_heaters(channel_voltage_pairs)
        
        print("Waiting 1 second for stabilization...")
        sleep(1.0)
        
        print("\nMeasurements for selected channels:")
        for channel, _ in channel_voltage_pairs:
            v, i, r, p = calculate_virp(channel)
            print(f"Channel {channel:2d}: V={v:.4f} V, I={i:.4f} A, R={r:.4f} Ω, P={p:.4f} W")

def temperature_to_resistance(M, C, temperature):
    return (temperature - C) / M

# CSV and real-time plots section
def initialise_csv(channel_numbers, device_names, filename=None, filename_prefix=None):

    '''
    Create CSV file with filename_prefix (optional), device names, channel numbers and datetime
    Create appropriate headers in CSV file to log multi-channel data.
    Args:
        channel_numbers (list)
        device_names (list): device names corresponding to each channel
        filename (str, optional): to reference CSV file
        filename_prefix (str, optional)
    '''

    # Validate that device_names matches channel_numbers
    if len(device_names) != len(channel_numbers):
        raise ValueError("device_names and channel_numbers must have the same length")
    
    # create filename with optional prefix, device-channel pairs and datetime
    filename_parts = []
    for device, ch in zip(device_names, channel_numbers):
        filename_parts.append(f"{device}-ch{ch}")
    
    # Build the filename with optional prefix
    base_filename = f"{'_'.join(filename_parts)}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    if filename_prefix is not None:
        filename = f"{filename_prefix}_{base_filename}"

    else:
        filename = base_filename

    # create headers
    headers = ["Time (s)"] # first column is always time

    # for each channel, add 4 measurement columns
    for device, ch in zip(device_names, channel_numbers):
        headers.extend([
            f"{device}_Ch{ch}_Voltage (V)", 
            f"{device}_Ch{ch}_Current (A)", 
            f"{device}_Ch{ch}_Resistance (Ω)", 
            f"{device}_Ch{ch}_Power (W)"
        ])
        
    # create new CSV file with mode = 'w', and write headers
    with open(filename, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
    print(f"CSV file initialised: {filename}")
    return filename

def save_data_to_csv(data, filename, channel_numbers):
    '''
    Appends latest data point from each channel and 
    writes it as single row to CSV file.
    Args:
        data (dict): Nested dictionary containing data:
        {
            "times": [...],
            "ch_1": {"voltages": [...], "currents": [...], ...},
            "ch_2": {"voltages": [...], "currents": [...], ...},
            ...
        }
        filename (str): path to CSV file to append data to
        channel_numbers (list)

    '''
    with open(filename, mode= "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        # if we have time data (measurements exist), build row with timestamp
        if data["times"]:
            latest_time = data["times"][-1]
            row = [latest_time]

            # get most recent measurements for each channel            
            for ch in channel_numbers:

                # get channel's data dictionary
                ch_data = data[f"ch_{ch}"]

                # append only most recent measurements
                row.extend([
                    ch_data["voltages"][-1],
                    ch_data["currents"][-1], 
                    ch_data["resistances"][-1],
                    ch_data["powers"][-1]
                    ])
                
            # write complete row to CSV file
            writer.writerow(row)

def initialise_plots(channel_numbers, device_names):
    '''
    create individual plot for single channel or overlay plots for multiple channels
    Args:
    channel_numbers (list)
    device_names (list): device names corresponding to each channel
    Returns:
        tuple: (fig, axs, data, lines) where:
            fig: matplotlib figure object
            axs: array of subplot axes, always 4 subplots in 2x2 grid
            data: nested dictionary
            lines: matplotlib line objects
    '''
    # Validate device_names
    if len(device_names) != len(channel_numbers):
        raise ValueError("device_names and channel_numbers must have the same length")

    plt.ion()  # Enable interactive mode for real-time updates
    
    n_channels = len(channel_numbers)
    
    if n_channels == 1:
        # Single channel: 4 subplots
        fig, axs = plt.subplots(2, 2, figsize=(10, 8))
        axs = axs.ravel()  # flatten 2D array to 1D for easy indexing
    else:
        # Multiple channels: overlay all channels on same 4 subplots
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        axs = axs.ravel()  # flatten 2D array to 1D for easy indexing

    # Center the plot window on screen
    try:
        manager = plt.get_current_fig_manager()
        # Try different backend methods to center window
        if hasattr(manager, 'window'):
            # For TkAgg backend
            manager.window.wm_geometry("+%d+%d" % (
                (manager.window.winfo_screenwidth() - 9*80) / 2,
                (manager.window.winfo_screenheight() - 5*80) / 2
            ))
    except:
        pass  # If centering fails, continue without error

    # Initialise data structures with start_time tracker
    # start_time will be set to None initially, then set to first timestamp
    data = {"times": [], "start_time": None}
    lines = {}
    
    colors = ['r-', 'g-', 'b-', 'm-', 'c-', 'y-', 'k-', 'orange', 'purple', 'brown']
    
    for i, (ch, device) in enumerate(zip(channel_numbers, device_names)):
        # Initialize empty data lists for this channel
        data[f"ch_{ch}"] = {
            "voltages": [],
            "currents": [],
            "resistances": [],
            "powers": []
        }
        
        color = colors[i % len(colors)]
        label = f"{device}_Ch{ch}"
        
        # create empty plot lines for each measurement
        # all channels use same 4 subplots
        lines[f"ch_{ch}_voltage"], = axs[0].plot([], [], color, label=label)
        lines[f"ch_{ch}_current"], = axs[1].plot([], [], color, label=label)
        lines[f"ch_{ch}_resistance"], = axs[2].plot([], [], color, label=label)
        lines[f"ch_{ch}_power"], = axs[3].plot([], [], color, label=label)
    
    # Configure subplot appearance and labels
    axs[0].set_title("Voltage", fontsize=10)
    axs[0].set_xlabel("Time (s)", fontsize=10)
    axs[0].set_ylabel("Voltage (V)", fontsize=10)
    axs[0].grid(True, alpha=0.3)
    axs[0].tick_params(axis='both', which='major', labelsize=9)
    
    axs[1].set_title("Current", fontsize=10)
    axs[1].set_xlabel("Time (s)", fontsize=10)
    axs[1].set_ylabel("Current (mA)", fontsize=10)
    axs[1].grid(True, alpha=0.3)
    axs[1].tick_params(axis='both', which='major', labelsize=9)
    
    axs[2].set_title("Resistance", fontsize=10)
    axs[2].set_xlabel("Time (s)", fontsize=10)
    axs[2].set_ylabel("Resistance (Ω)", fontsize=10)
    # set initial y-limits for resistance
    # axs[2].set_ylim(0,500) 
    axs[2].grid(True, alpha=0.3)
    axs[2].tick_params(axis='both', which='major', labelsize=9)
    
    axs[3].set_title("Power", fontsize=10)
    axs[3].set_xlabel("Time (s)", fontsize=10)
    axs[3].set_ylabel("Power (W)", fontsize=10)
    axs[3].grid(True, alpha=0.3)
    axs[3].tick_params(axis='both', which='major', labelsize=9)
    
    # Add legends for multiple channels
    if n_channels > 1:
        for ax in axs:
            ax.legend(fontsize=9)

    # subplot spacing and display
    plt.tight_layout()
    fig.canvas.draw()
    fig.show()

    return fig, axs, data, lines

def update_plots(axs, data, lines, channel_numbers, measurements, last_plot_time, plot_interval=2.0):
    '''
    Updates real-time plots with new measurement data from all channels.
    Only refreshes plots every plot_interval=2.0s to prevent excessive CPU usage.
    Returns:
        last_plot_time (float): updated time of when plots were refreshed
    '''

    # get current time for this measurement cycle
    current_time = time()

    # Set start_time on first measurement and normalize time to start at 0s
    if data["start_time"] is None:
        data["start_time"] = current_time
    
    # Store normalised time (relative to start_time, starting at 0 s)
    normalised_time = current_time - data["start_time"]
    data["times"].append(normalised_time)   # add normalised time to shared time array

    # store new measurements in data structure for each channel
    for i, ch in enumerate(channel_numbers):
        # extract measurements for channel
        v, i_curr, r, p = measurements[i]

        # get reference to channel's data dictionary
        ch_data = data[f"ch_{ch}"]

        # append new measurements to channel's data arrays
        ch_data["voltages"].append(v)
        ch_data["currents"].append(i_curr)
        ch_data["resistances"].append(r)
        ch_data["powers"].append(p)

   # Always update on first data point (len == 1) or after plot_interval=2.0s
    if current_time - last_plot_time >= plot_interval or len(data["times"]) == 1:

        for i, ch in enumerate(channel_numbers):    
            ch_data = data[f"ch_{ch}"]   # get channel's data

            # update plot lines with x=times, y=measurements            
            lines[f"ch_{ch}_voltage"].set_data(data["times"], ch_data["voltages"])
            lines[f"ch_{ch}_current"].set_data(data["times"], ch_data["currents"])
            lines[f"ch_{ch}_resistance"].set_data(data["times"], ch_data["resistances"])
            lines[f"ch_{ch}_power"].set_data(data["times"], ch_data["powers"])
            
        # Update axis limits for 4 subplots to accommodate new data
        # Update axis limits for voltage, current, and power subplots
        for idx, ax in enumerate(axs):
            if idx == 2:  # Resistance subplot
                # For resistance: only rescale if all values are within valid range
                all_resistances = []
                for ch in channel_numbers:
                    all_resistances.extend(data[f"ch_{ch}"]["resistances"])
                
                # Check if any resistance is negative or > 550
                has_invalid = any(r < -50 or r > 550 for r in all_resistances)
                
                if not has_invalid:
                    # Safe to rescale - all values are valid
                    ax.relim()
                    ax.autoscale_view()
                    
                else:
                    # Do NOT rescale, but ensure x-axis (time) still updates
                    ax.relim()
                    ax.autoscale_view(scalex=True, scaley=False)
            else:
                # For voltage, current, power: always rescale normally
                ax.relim()
                ax.autoscale_view()

        # brief pause for GUI to update and remain responsive
        plt.pause(0.01)

        # update time tracking when plot was last refreshed
        last_plot_time = current_time

    return last_plot_time

# Feedback Control Section
def feedback_loop(channel_numbers, target_resistances, tolerance_resistance,
                  fail_resistance, update_time=2.0, voltage_step=0.1,
                  filename=None, filename_prefix=None,
                  hold_duration=None,
                  fig=None, axs=None, data=None, lines=None, device_names=None,
                  failed_channels_set=None
                  ):
    
    """
    Continuously adjust voltage to maintain target resistance for one or multiple channels
    Failed channels are set to 0 V and excluded from control (persist across loops)
    Monitor channels with None target resistance are set to 0 V once and excluded from control (only in current loop)
    All active channels, EXCEPT failed and monitor channels, must reach targets before hold timer starts

    Args:
        channel_numbers (list)
        target_resistances (list): use None to exclude a channel
        tolerance_resistance (float): acceptable deviation from target (±Ω), same for ALL channels
        fail_resistance (float): same for ALL channels
        update_time (float): time interval between measurements and adjustments (s), same for ALL channels
        voltage_step (float): increase/decrease in voltage per update time, same for ALL channels
        filename (str): set to None for first feedback loop
        filename_prefix (str)
        hold_duration (float): duration to maintain target(s) before next feedback sequence(s); set to None for continuous operation
        fig, axs, data, lines: Matplotlib objects for continuing real-time plots from previous sequences
        device_names: paired to channel_numbers for real-time plot labels and CSV headers
        failed_channels_set: created to persist across loops
    
    Returns:
        last_voltages (dict): voltages (V) for each channel
        last_resistances (dict): resistances (Ω) for each channel
        CSV filename (str): used for subsequent loops
        fig, axs, data, lines: Matplotlib objects for continuing real-time plots
    """
    
    # validate input parameters
    if len(channel_numbers) != len(target_resistances):
        raise ValueError("channel_numbers and target_resistances must have the same length")

    if device_names is not None and len(device_names) != len(channel_numbers):
        raise ValueError("device_names and channel_numbers must have the same length")
    
    # Initialise csv filename for data logging
    # If first call, create new file; otherwise append to existing file from previous sequence
    if filename is None:
        filename = initialise_csv(channel_numbers, device_names, filename=None, filename_prefix=filename_prefix)
    else:
        print(f"Appending data to existing file: {filename}")

    # Initialise or continue real-time plots
    # If first call, create new plots; otherwise append to existing plots from previous sequence
    if fig is None or axs is None or data is None or lines is None:
        plt.ion() # enable interactive plotting mode
        fig, axs, data, lines = initialise_plots(channel_numbers, device_names)
    
    # ensure first plot happens immediately
    last_plot_time = 0  

    # dictionaries to track most recent voltage and resistance for each channel
    # persist across loop iterations to enable voltage adjustments
    last_voltages = {}
    last_resistances = {}

    # initialise voltages and resistances for each channel 
    for ch in channel_numbers:
        v, i, r, p = calculate_virp(ch)
        last_voltages[ch]= v
        last_resistances[ch] = 0 # will be updated in first measurement cycle

    # timer for tracking time passed after all channels have maintained target resistances
    # remains None until all channels first reach their target resistances
    hold_start_time = None

    # track which channels have failed (persists across loops)
    if failed_channels_set is None:
        failed_channels_set = set()

    # track channels with None target resistance (resets every loop)
    excluded_channels_set = set()

    # add channels with -1 target resistance to ramp down afterwards
    rampdown_channels_set = set()

    # check if ALL non-None channels are set to -1
    is_rampdown_mode = all(
        target_resistances[i] == -1
        for i, ch in enumerate(channel_numbers)
        if target_resistances[i] is not None
    )

    channels_to_exclude = []
    for i, ch in enumerate(channel_numbers):

        target_r = target_resistances[i]

        if target_r is None:
            excluded_channels_set.add(ch)
            # sent as [(chX, vX = 0), chY, vY = 0, ...] to Teensy
            channels_to_exclude.append((ch, 0))
            last_voltages[ch] = 0
            print(f"Channel {ch} target resistance is None. Setting to 0 V and excluding from feedback control.")
        
        elif target_r == -1:
            rampdown_channels_set.add(ch)
            print(f"Channel {ch} set to ramp down to 0 V.")
        
    # batch set excluded channels to 0 V at once
    if channels_to_exclude:
        teensy_set_heaters(channels_to_exclude)

    # MAIN FEEDBACK LOOP
    try:
        while True:

            measurements = []
            for ch in channel_numbers:
                v, i, r, p = calculate_virp(ch)
                measurements.append((v, i, r, p))
                last_resistances[ch] = r # update resistance tracking

            last_plot_time = update_plots(axs, data, lines, channel_numbers, measurements, last_plot_time)
            save_data_to_csv(data, filename, channel_numbers)

            # triggered when all channels reach target resistance
            # determines when to start hold timer
            all_targets_reached = True

            # collect all channels requiring voltage updates for batch transmission
            # to minimise communication overhead with Teensy
            channels_to_update = []

            # collect all channels that failed so that Teensy can batch set them to 0 V
            failed_channels = []

            # iterate through channel numbers and keeps track of their index in the list
            for i, ch in enumerate(channel_numbers):
                # in each tuple, only take voltage and resistance
                v_measured, _, r, _ = measurements[i]

                # get target resistances corresponding to channels
                target_r = target_resistances[i]

                # skip excluded channels with None target resistances
                if ch in excluded_channels_set:
                    continue

                # check for new channel that failed in current iteration
                # v_measured > 1.2 V is to ensure failure is only checked after new channel(s) is/are initialised
                if r > fail_resistance and v_measured > 1.2:
                    if ch not in failed_channels_set:
                        print(f"Channel {ch} resistance too high. Voltage set to 0 V.")
                    
                    failed_channels.append((ch, 0)) # queue for Teensy batch update
                    last_voltages[ch] = 0 # update voltage tracking
                    failed_channels_set.add(ch) # persistently mark as failed
                    continue # skip new failed channel in THIS iteration

                # skip any channels that failed in previous iterations
                if ch in failed_channels_set:
                    continue

                # RAMP DOWN CONTROL
                # All non-None channels set to target_r = -1
                if ch in rampdown_channels_set:
                    if last_voltages[ch] > 0.001:
                        # ramp down voltage
                        new_v = round(max(0, last_voltages[ch] - voltage_step), 4)

                        # if new voltage is below 1 mV, set to exactly zero
                        if new_v < 0.001:
                            new_v = 0

                        channels_to_update.append((ch, new_v))
                        last_voltages[ch] = new_v
                        all_targets_reached = False # still ramping down

                    # else: channel is below 1 mV 
                    continue

                # MAIN FEEDBACK CONTROL
                # skips failed channels and excluded channels with None target resistance
                # also skips if in rampdown mode
                delta = 0

                if r > target_r + tolerance_resistance:
                    delta = -voltage_step
                elif r < target_r - tolerance_resistance:
                    delta = voltage_step

                # only apply voltage adjustment if needed
                if delta != 0:
                    new_v = round(last_voltages[ch] + delta, 4)

                    # ensure voltage never goes below 0 V
                    if new_v < 0:
                        new_v = 0

                    channels_to_update.append((ch, new_v)) # queue for Teensy batch update

                    last_voltages[ch] = new_v # update tracked voltage
                            
                # if any active channel is outside tolerance, group condition is False
                # this check skips failed channels and channels with None target resistance
                if abs(r - target_r) > tolerance_resistance:
                    all_targets_reached = False
            
            # BATCH VOLTAGE UPDATES - minimise communication overhead and ensure synchronised control
            # batch send all voltage updates to Teensy if there are any changes
            if channels_to_update:
                teensy_set_heaters(channels_to_update)

            # batch set all failed channels to 0 V at once           
            if failed_channels:
                teensy_set_heaters(failed_channels)

            # HOLD TIMER MANAGEMENT
            # normal mode: start timer when all channels reach target resistance
            # ramp-down mode: start timer when all channels reach 0 V
            if all_targets_reached and hold_duration is not None:
                if hold_start_time is None:
                    hold_start_time = time()
                    if is_rampdown_mode:
                        print(f"All channels at 0 V. Exiting feedback loop in {hold_duration} s...")
                    else:
                        print(f"All target resistances reached, target resistances will be stabilised for {hold_duration} s...")
                else:
                    # check if hold duration has been reached
                    elapsed = time() - hold_start_time
                    if elapsed >= hold_duration:
                        print(f"Max duration of {hold_duration} s reached. Exiting this feedback sequence...")
                        break # exit feedback loop, starting the next sequence
            
            # WAIT BEFORE NEXT ITERATION
            sleep(update_time)

    except KeyboardInterrupt:
        print("\nFeedback loop stopped by user.")
        # set all channels to last known voltages
        channels_to_reset = [(ch, last_voltages[ch]) for ch in channel_numbers]
        teensy_set_heaters(channels_to_reset)
        
    # RETURN STATE FOR NEXT SEQUENCE
    return last_voltages, last_resistances, filename, fig, axs, data, lines, failed_channels_set

def series_feedback(channel_numbers, device_names, feedback_schedule, tolerance_resistance,
                    fail_resistance, filename=None, filename_prefix=None):
    """
    Run a series of feedback loops for multiple target resistances with specified hold durations.
    
    Args:
        channel_numbers (list): list of channel numbers
        device_names (list): device names corresponding to each channel
        feedback_schedule (list of tuples): [
            ([targetR1, targetR2, ...], hold_duration, update_time, voltage_step), 
            ...]
            The final target should have hold_duration=None to indicate it runs indefinitely.
            Set targetR=None excludes channel from feedback control and hold timer start conditions.
        tolerance_resistance (float)
        fail_resistance (float)
        filename (str or None): existing filename to continue logging to
        filename_prefix (str or None)
    
    Returns:
        last_voltages (dict), last_resistances (dict), filename
    """

    # Validate device_names
    if len(device_names) != len(channel_numbers):
        raise ValueError("device_names and channel_numbers must have the same length")
    
    last_voltages = {}
    last_resistances = {}

    # initialise plotting objects to keep them across steps
    fig = axs = data = lines = None

    # track failed channels that persist across loops
    failed_channels_set = set()
    
    try:
        for i, (step_target_resistances, hold_duration, update_time, voltage_step) in enumerate(feedback_schedule):

            # error handling
            if len(step_target_resistances) != len(channel_numbers):
                raise ValueError(
                    f"Step {i+1}: number of target resistances ({len(step_target_resistances)}) "
                    f"does not match number of channels ({len(channel_numbers)})."
                )
        
            print(f"\n--- Step {i+1}: Target resistance = {step_target_resistances} Ω ---")
            print(f"Update time = {update_time}s, Voltage step = {voltage_step}V")

            # to check if a duration is provided
            if hold_duration is not None:
                print(f"Holding target for {hold_duration} seconds after reaching it...")
        
            else:
                print("Final target: running feedback loop until keyboard interrupt.")

            # Run the feedback loop for this step
            last_voltages, last_resistances, filename, fig, axs, data, lines, failed_channels_set = feedback_loop(
                channel_numbers=channel_numbers,
                target_resistances=step_target_resistances,
                tolerance_resistance=tolerance_resistance,
                fail_resistance=fail_resistance,
                update_time=update_time,
                voltage_step=voltage_step,
                filename=filename,
                filename_prefix=filename_prefix,
                hold_duration=hold_duration,
                fig=fig,
                axs=axs,
                data=data,
                lines=lines,
                device_names=device_names,
                failed_channels_set=failed_channels_set
            )
        
            print(f"Step {i+1} complete. Voltages: {last_voltages}, Resistances: {last_resistances}")
        
    except KeyboardInterrupt:
        print("\n\nSeries feedback stopped by user.")
        print(f"Last state - Voltages: {last_voltages}, Resistances: {last_resistances}")

    finally:
        plt.ioff()

        if fig is not None:
            try: 
                fig.canvas.draw()
                fig.canvas.flush_events()
            except: 
                pass

    return last_voltages, last_resistances, filename