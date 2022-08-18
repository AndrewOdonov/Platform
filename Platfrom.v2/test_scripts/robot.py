import serial


def configure_serial_port():
    """
    set port and rate of transmitting data
    :return: settings
    """
    settings = serial.Serial('/dev/ttyUSB0')
    settings.baudrate = 115200
    return settings

def read_data_from_sensors(settings):
    """
    read and decode data from module of control
    :return: string_data
    """
    try:
        data_from_serial = settings.readline()
        string_data = data_from_serial.decode()
        return string_data
    except Exception:
        return None

def parse_data(string_data):
    """
    The function parses data from module of control and create dictionary data from sensors
    :return: device_name_str, device_data, data_ID
    """
    device_name_str, device_data_str = string_data[1:-1].split('#')
    device_data = device_data_str.split('/')
    data_ID = {device_name_str: device_data}
    return device_name_str, device_data, data_ID


