
# Lab 0x06: JSON storage used at startup, before the scheduler starts.
# Motors remain disabled during calibration. Startup code handles file errors.
# calibration is a dictionary with three lists or tuples of length 13:
# "channels": printed sensor numbers in driver order (not MCU pin names).
# "black": measured average ADC counts on black, in that same order.
# "white": measured average ADC counts on white, in that same order.
# Store ADC counts (0-4095), not normalized weights. Convert ulab arrays to
# ordinary lists with .tolist() before storing them. JSON loads arrays as lists.
#from fileinput import filename
import json
# from sklearn import calibration



def validate_calibration(calibration, channels, min_span, adc_max=4095):
    try:
        white, black = calibration["white"], calibration["black"]
        saved_channels = calibration["channels"]
    except (KeyError, TypeError) as error:
        raise ValueError("Calibration is missing required channel data") from error
    
    if len(channels) != 13 or list(saved_channels) != list(channels):
        raise ValueError("Calibration must match all 13 configured channels")
    
    if len(white) != 13 or len(black) != 13 or not 0 < min_span <= adc_max:
        raise ValueError("Invalid calibration lengths or minimum span")
    
    try:
        for w, b in zip(white, black):
            if not 0 <= w < b <= adc_max or b - w < min_span:
                raise ValueError("Invalid black/white values or insufficient contrast")
    except TypeError as error:
        raise ValueError("Calibration values must be numeric") from error
        
    return calibration


def save_calibration(filename, calibration, channels, min_span, adc_max=4095):
    # Validate before opening the destination, then write the JSON file.
    validate_calibration(calibration, channels, min_span, adc_max)
    
    with open(filename, "w") as file:
        json.dump(calibration, file)
        
        
def load_calibration(filename, channels, min_span, adc_max=4095):
    # Read and validate; startup code reports errors or repeats calibration.
    
    with open(filename, "r") as file:
        calibration = json.load(file)
        
    validate_calibration(calibration, channels, min_span, adc_max)
    
    return calibration


   
# Usage in your startup code, AFTER measuring all channels on black, then white:
# from line_calibration import save_calibration, load_calibration
# channels = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
# Reorder channels if your driver reads them in a different order.
# black_averages and white_averages each contain 13 measured numbers, ordered
# exactly like channels; they come from your captures, not from this helper.
# calibration = {
# "channels": channels,
# "black": list(black_averages),
# "white": list(white_averages),
# }
# min_span = 10 # Example ADC-count threshold; choose for your contrast/noise.
# save_calibration("line_calibration.json", calibration, channels, min_span)
# At a later startup, reuse the file only if sensing conditions are unchanged:
# calibration = load_calibration("line_calibration.json", channels, min_span)
# white = calibration["white"] # white[i] belongs to channels[i].
# black = calibration["black"] # black[i] belongs to that same channels[i].
