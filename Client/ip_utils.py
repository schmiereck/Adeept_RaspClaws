"""
Utility functions for IP address management
"""

IP_TXT_PATH = './IP.txt'


def num_import(initial):
    """Import IP address from IP.txt file

    Args:
        initial: The prefix to search for (e.g., 'IP:')

    Returns:
        The value after the prefix, stripped of whitespace
    """
    with open(IP_TXT_PATH) as f:
        for line in f.readlines():
            if line.find(initial) == 0:
                r = line
                break
        else:
            raise ValueError(f"Could not find '{initial}' in {IP_TXT_PATH}")

    begin = len(list(initial))
    snum = r[begin:]
    n = snum.strip()
    return n


def replace_num(initial, new_num):
    """Replace data in IP.txt file

    Args:
        initial: The prefix to search for (e.g., 'IP:')
        new_num: The new value to set
    """
    newline = ""
    str_num = str(new_num)
    with open(IP_TXT_PATH, "r") as f:
        for line in f.readlines():
            if line.find(initial) == 0:
                line = initial + "%s" % (str_num)
            newline += line
    with open(IP_TXT_PATH, "w") as f:
        f.writelines(newline)


def get_ip_address():
    """Get IP address from IP.txt file

    Returns:
        The IP address as string
    """
    return num_import('IP:')
