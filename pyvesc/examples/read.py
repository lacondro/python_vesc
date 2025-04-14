import pyvesc
from pyvesc.VESC.messages import (
    GetValues,
    SetRPM,
    SetCurrent,
    SetRotorPositionMode,
    GetRotorPosition,
)
import serial
import time

# Set your serial port here
serialport = "/dev/cu.usbmodem3041"  # For macOS

# Global variable to track serial connection status
serial_status = True


def close_serial_port(ser):
    """Helper function to safely close a serial port"""
    try:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
    except Exception as e:
        print(f"Error closing serial port: {e}")


def get_realtime_data(ser):
    """
    Requests and decodes VESC telemetry data using an existing serial connection.

    Args:
        ser: An open pyserial Serial object connected to the VESC.

    Returns:
        A VESC message object (e.g., GetValues) containing the data, or None if an error occurs.
    """
    global serial_status
    if ser is None or not ser.is_open:
        serial_status = False
        return None

    try:
        # Send request
        ser.write(pyvesc.encode_request(GetValues))

        # Try to read the response - adjust buffer size if needed, 128 is often enough for GetValues
        # A more robust method might involve checking ser.in_waiting and reading in chunks,
        # but pyvesc.decode expects a buffer it can consume from.
        buffer = ser.read(128)

        if buffer:
            (response, consumed) = pyvesc.decode(buffer)
            if consumed > 0:  # Check if decode was successful
                serial_status = True
                # print(response) # Optional: for debugging
                return response
            else:
                # Decode failed or returned no meaningful data
                print("Warning: VESC decode consumed 0 bytes.")
                serial_status = False
                return None
        else:
            # Read timed out or returned nothing
            # print("Warning: VESC read returned no data.") # Can be noisy
            serial_status = False
            return None

    except serial.SerialException as e:
        print(f"Serial Error during read: {e}")
        serial_status = False
        close_serial_port(ser)  # Attempt to close the problematic port
        return None
    except Exception as e:
        # Catch other potential errors (e.g., pyvesc decode issues, attribute errors)
        print(f"Error processing VESC data: {e}")
        serial_status = False
        return None


def get_rotor_position(ser):
    """
    Gets the rotor position from the VESC.

    Args:
        ser: An open serial connection to the VESC.

    Returns:
        Rotor position value or None if unsuccessful.
    """
    try:
        ser.write(pyvesc.encode_request(GetRotorPosition))
        buffer = ser.read(
            64
        )  # Smaller buffer might be sufficient for just the position

        if buffer:
            (response, consumed) = pyvesc.decode(buffer)
            if consumed > 0 and hasattr(response, "rotor_pos"):
                return response.rotor_pos
    except Exception as e:
        print(f"Error getting rotor position: {e}")

    return None


def monitor_vesc_data():
    """
    Opens a connection to the VESC and continuously monitors and displays the data.
    """
    try:
        # Open serial connection
        with serial.Serial(serialport, baudrate=115200, timeout=0.5) as ser:
            print(f"Connected to VESC on {serialport}")

            # Optional: Enable encoder mode
            # ser.write(
            #     pyvesc.encode(
            #         SetRotorPositionMode(SetRotorPositionMode.DISP_POS_MODE_ENCODER)
            #     )
            # )
            # time.sleep(0.2)  # Short delay

            # Set a small current to ensure motor is active
            # ser.write(pyvesc.encode(SetCurrent(0.0)))
            # time.sleep(0.2)

            print("Starting data monitoring...")

            while True:
                # Clear input buffer to avoid stale data
                ser.reset_input_buffer()

                # Get and display VESC values
                values = get_realtime_data(ser)
                if values:
                    print("\n--- VESC Values ---")
                    # print(f"RPM: {values.rpm}")
                    print(f"Current Motor: {values.avg_motor_current:.2f} A")
                    print(f"Current Input: {values.avg_input_current:.2f} A")
                    print(f"Duty Cycle: {values.duty_cycle_now:.2f}")
                    print(f"Temperature MOSFET: {values.temp_fet:.1f}°C")
                    print(f"Temperature Motor: {values.temp_motor:.1f}°C")
                    print(f"Voltage Input: {values.v_in:.2f} V")
                    print(f"Amp Hours: {values.amp_hours:.3f} Ah")
                    print(f"Amp Hours Charged: {values.amp_hours_charged:.3f} Ah")
                    print(f"Watt Hours: {values.watt_hours:.3f} Wh")
                    print(f"Watt Hours Charged: {values.watt_hours_charged:.3f} Wh")
                    print(f"Tachometer: {values.tachometer}")
                    print(f"Tachometer ABS: {values.tachometer_abs}")
                    print(
                        f"Fault Code: {values.mc_fault_code}"
                    )  # CORRECTED LINE                else:
                    # print("Failed to get VESC values")

                # Clear input buffer again
                ser.reset_input_buffer()

                # Get and display rotor position
                rotor_pos = get_rotor_position(ser)
                if rotor_pos is not None:
                    print(f"Rotor Position: {rotor_pos:.4f}")

                # print("Setting Current to 1.0A")
                # ser.write(pyvesc.encode(SetCurrent(1.0)))
                # Alternate between different motor commands every 5 seconds
                # if time.time() % 10 < 5:
                #     print("Setting RPM to 2000")
                #     ser.write(pyvesc.encode(SetRPM(0)))
                # else:
                #     print("Setting Current to 1.0A")
                #     ser.write(pyvesc.encode(SetCurrent(1.0)))

                time.sleep(0.5)  # Update interval

    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
    # except serial.SerialException as e:
    #     print(f"Serial error: {e}")
    # except Exception as e:
    #     print(f"Unexpected error: {e}")
    finally:
        # Try to ensure motor is stopped if the program exits
        try:
            with serial.Serial(serialport, baudrate=115200, timeout=0.1) as ser:
                ser.write(pyvesc.encode(SetCurrent(0)))
                print("Motor current set to 0")
        except:
            pass


if __name__ == "__main__":
    monitor_vesc_data()
