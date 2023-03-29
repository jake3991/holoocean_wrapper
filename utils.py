import numpy as np
from scipy.interpolate import interp1d

def generate_map(config: dict) -> np.array:
    """Generate a map to convert a holoocean sonar image from 
    polar to cartisian coords. Note this does not apply the remapping
    it generates the tools to apply cv2.remap later. 

    Args:
        config (dict): the holoocean config file. Read in using
        holoocean.packagemanager.get_scenario(scenario)

    Returns:
        np.array: the numpy arrays to apply cv2 remap to a sonar image
    """

    # parse out the parameters
    config = config['agents'][0]['sensors'][-1]["configuration"]
    horizontal_fov = float(config['Azimuth'])
    range_resolution = (config['RangeMax'] - config['RangeMin']) / 512
    height = range_resolution * config['RangeBins']
    rows = config['RangeBins']
    width = np.sin(np.radians(horizontal_fov / 2)) * height * 2
    cols = int(np.ceil(width / range_resolution))
    bearings = np.radians(np.linspace(-horizontal_fov/2, horizontal_fov/2, config['AzimuthBins']))
    
    # create an interpolation object for bearing angle
    f_bearings = interp1d(
                bearings,
                range(len(bearings)),
                kind='linear',
                bounds_error=False,
                fill_value=-1,
                assume_sorted=True)

    #build the meshgrid
    XX, YY = np.meshgrid(range(cols), range(rows))
    x = range_resolution * (rows - YY)
    y = range_resolution * (-cols / 2.0 + XX + 0.5)
    b = np.arctan2(y, x) * -1
    r = np.sqrt(np.square(x) + np.square(y))
    map_y = np.asarray(r / range_resolution, dtype=np.float32)
    map_x = np.asarray(f_bearings(b), dtype=np.float32)

    return map_x, map_y


def parse_keys(keys: list, val : int, depth_command : float) -> np.array:
    """Handle any pressed keys on the keyboard, converting them into 
    commands for the ROV. 

    Args:
        keys (list): list of pressed keys on the keyboard
        val (int): the force strenght we apply to the command array
        depth_command (float): the depth we want the ROV at

    Returns:
        np.array: an array of the commands to be sent to the ROV
    """


    command = np.zeros(8) # the commands start out as all zeros
    command[0:4] = depth_command # set the depth command

    if 'i' in keys: # up
        command[0:4] += val
    if 'k' in keys: # down
        command[0:4] -= val
    if 'j' in keys: # rotate left
        command[[4,7]] += val
        command[[5,6]] -= val
    if 'l' in keys: # rotate right 
        command[[4,7]] -= val
        command[[5,6]] += val
    if 'w' in keys: # forward
        command[4:8] += val
    if 's' in keys: # backward
        command[4:8] -= val
    if 'a' in keys: # strafe left
        command[[4,6]] += val
        command[[5,7]] -= val
    if 'd' in keys: #strafe right
        command[[4,6]] -= val
        command[[5,7]] += val

    return command