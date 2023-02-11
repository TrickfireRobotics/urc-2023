import rssi


def isSignalGood(stringSignal):
    signal = int(stringSignal)
    return signal > -100


def checkSignal():
    interface = 'wlp1s0'
    rssi_scanner = rssi.RSSI_Scan(interface)

    ssids = ['trickfire']

    # sudo argument automatixally gets set for 'false', if the 'true' is not set manually.
    # python file will have to be run with sudo privileges.
    ap_info = rssi_scanner.getAPinfo(networks=ssids, sudo=True)
    # check if signal is good
    return isSignalGood(ap_info[0])


def main():
    while (true):
        checkSignal()
    # check the signal if it good
    # if it's not good move the arms

    print('Hi from antenna.')


if __name__ == '__main__':
    main()
