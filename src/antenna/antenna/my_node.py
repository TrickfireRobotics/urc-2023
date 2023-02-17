import rssi
import time
import robotInterface


def isSignalGood():
    interface = 'wlp1s0'
    rssi_scanner = rssi.RSSI_Scan(interface)

    ssids = ['trickfire']

    # sudo argument automatixally gets set for 'false', if the 'true' is not set manually.
    # python file will have to be run with sudo privileges.
    ap_info = rssi_scanner.getAPinfo(networks=ssids, sudo=True)
    signal = int(ap_info[0])

    # check if signal is good
    return signal > -100


def turnAntenna():
    turnDegree = 0
    # turn right if the signal bad and until get to 180 degree to the right
    while (isSignalGood() != True & turnDegree < 90):
        turnDegree += 10
        robotInterface.antennaTurnRight(turnDegree)

    # if after turn all to the right but still don't have the good signal turn left
    if (isSignalGood() != True):
        turnDegree = 0
        # turn left until the signal good or turn to 180 degree to the left
        # turnDegree less than 360 degree because the first 180 degree
        # is turn back to the begin after turn right
        while (isSignalGood() != True & turnDegree < 180):
            turnDegree += 10
            robotInterface.antennaTurnLeft(turnDegree)


def main():
    while (true):
        if (isSignalGood() != True):
            print("not good")
        turnAntenna()
        time.sleep(0.2)
    # check the signal if it good
    # if it's not good move the arms

    print('Hi from antenna.')


if __name__ == '__main__':
    main()
