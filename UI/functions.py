import os


def getPorts():
    """Used to gather every connected devices on usb.

    returns: list of found devices on COM ports.
    """
    arduinoPorts = os.popen(
        "python -m serial.tools.list_ports").read().strip().replace(' ', '').split('\n')
    if arduinoPorts == ['']:
        return []
    return arduinoPorts
