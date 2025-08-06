import time
import argparse
import caproto as ca
from caproto.threading.client import Context


def parse_cli():
    parser = argparse.ArgumentParser(description='Archon Interal TS mode syncing.')

    parser.add_argument('cam',
                        help='PV base for the camera')

    parser.add_argument('-t',
                        '--tpr',
                        metavar=('TPR_PV', 'TPR_CH'),
                        nargs=2,
                        help='PV base of tpr if IOC uses tpr instead of an evr')

    parser.add_argument('--timeout',
                        type=float,
                        default=10.0,
                        help='General timeout value in seconds used ca (default = 10.0s)')

    return parser.parse_args()


def generate_pv_list(CAM, TPR=None):
    pvlist = {
        'ts_policy': f"{CAM}:TSS:TsPolicy",
        'ts_status': f"{CAM}:TSS:SyncStatus",
        'intreq': f"{CAM}:TSS:IntReq",
        'running': f"{CAM}:DetectorState_RBV",
        'ls_mode': f"{CAM}:ArchonLineScanMode",
        'ls_mode_rbv': f"{CAM}:ArchonLineScanMode_RBV",
        'acquire': f"{CAM}:Acquire",
    }

    if TPR is not None:
        TPR_PV, TPR_CH = TPR
        pvlist['tpr'] = {
            'lcls_mode': f"{TPR_PV}:MODE",
            'ratemode': f"{TPR_PV}:CH{TPR_CH}_RATEMODE",
            'fixedrate': f"{TPR_PV}:CH{TPR_CH}_FIXEDRATE",
            'evcode': f"{TPR_PV}:CH{TPR_CH}_EVCODE",
        }
    else:
        pvlist['evr'] = {
            'evcode': f"{CAM}:CamEventCode",
            'evcode_rbv': f"{CAM}:CamEventCode_RBV",
        }

    return pvlist


class PvManager:
    def __init__(self, pvlist, timeout):
        self._timeout = timeout
        self.ctx = Context(timeout=timeout)
        self.pvs = self.get_pvs(pvlist)

    def __contains__(self, name):
        return name in self.pvs


    def _unpack(self, names):
        pvkey = self.pvs
        for name in names:
            pvkey = pvkey[name]

        return pvkey

    def read(self, *names):
        res = self._unpack(names).read(data_type='control')
        if res.data_count == 1:
            if res.data_type == ca.ChannelType.CTRL_ENUM:
                return res.metadata.enum_strings[res.data[0]].decode('utf8')
            else:
                return res.data[0]
        else:
            return res.data

    def write(self, *names, value, wait=True):
        self._unpack(names).write([value],
                                  data_type=ca.ChannelType.STRING if isinstance(value, str) else 'native',
                                  wait=wait)

    def wait(self, *names, value, timeout=None):
        if timeout is None:
            timeout = self._timeout

        start = time.time()

        while (time.time() - start < timeout):
            rv = self.read(*names)
            if rv == value:
                return True

        return False

    def get_pvs(self, pvlist):
        pvs = {}

        for name, pv in pvlist.items():
            if isinstance(pv, dict):
                pvs[name] = self.get_pvs(pv)
            else:
                pvs[name] = self.ctx.get_pvs(pv)[0]

        return pvs


def main():
    args = parse_cli()

    pvlist = generate_pv_list(args.cam, args.tpr)
    pvm = PvManager(pvlist, args.timeout)

    ts_policy = pvm.read('ts_policy')
    ts_status = pvm.read('ts_status')
    ls_mode = pvm.read('ls_mode_rbv')

    """
    Check the current sync policy and status. We only try to sync if 
    its set to "INTERNAL" mode and in the "Unlocked" state.
    """
    if ts_policy == "INTERNAL" and ts_status == "Unlocked":
        print(f"Attempting to sync internal timestamps for {args.cam}")
        lcls_mode = pvm.read('tpr', 'lcls_mode')
        if 'tpr' in pvm:
            if lcls_mode == "SC":
                # Save the values we will overwrite to restore them later.
                old_rate_mode = pvm.read('tpr', 'ratemode')
                old_fixed_rate = pvm.read('tpr', 'fixedrate')
                # Set the trigger rate to fixed rate 1Hz
                pvm.write('tpr', 'ratemode', value="Fixed")
                pvm.write('tpr', 'fixedrate', value="1Hz")
            else:
                # Save the values we will overwrite to restore them later.
                old_evcode = pvm.read('tpr', 'evcode')
                # Set the trigger to event code 45
                pvm.write('tpr', 'evcode', value=45)
        else:
            # Save the old event code to restore later.
            old_evt_code = pvm.read('evr', 'evcode_rbv')
            # Set the trigger to event code 45
            pvm.write('evr', 'evcode', value=45)

        # If the archon is running stop it before changing linescan mode to disabled
        if pvm.read('running') != "Idle" and ls_mode == "Enable":
            pvm.write('acquire', value="Done")
            pvm.wait('running', value="Idle")

        # Set linescan mode to disabled
        pvm.write('ls_mode', value="Disable")

        # If the archon is not running then re-enable acquisition
        if pvm.read('running') != "Acquire":
            # write always hits timeout on this pv even though the write happens...
            pvm.write('acquire', value="Acquire", wait=False)
            pvm.wait('running', value="Acquire")

        time.sleep(2.)
        # Initiate the sync.
        pvm.write('intreq', value=1)
        time.sleep(2.)
        # Check if the sync worked
        ts_status = pvm.read('ts_status')
        if ts_status == "Locked":
            print(f"Successfully synced {args.cam}")
        else:
            print(f"Failed to sync {args.cam} - check that there is triggers/timing")

        # Check if linescan mode needs to be restored
        if pvm.read('ls_mode_rbv') != ls_mode:
            pvm.write('acquire', value="Done")
            pvm.wait('running', value="Idle")
            pvm.write('ls_mode', value=ls_mode)

        if 'tpr' in pvm:
            # Restore the values we overwrote before.
            if lcls_mode == "SC":
                print(f"Restoring {args.tpr[0]}:CH{args.tpr[1]} to old settings")
                pvm.write('tpr', 'ratemode', value=old_rate_mode)
                pvm.write('tpr', 'fixedrate', value=old_fixed_rate)
            else:
                print(f"Restoring {args.tpr[0]}:CH{args.tpr[1]}_EVCODE to {old_evcode}")
                pvm.write('tpr', 'evcode', value=old_evcode)
        else:
            # Restore the event code we overwrote before.
            pvm.write('evr', 'evcode', value=old_evt_code)

        # If the archon is not running then re-enable acquisition
        if pvm.read('running') != "Acquire":
            # sleep a bit after restoring evrs
            time.sleep(2.5)
            # write always hits timeout on this pv even though the write happens...
            pvm.write('acquire', value="Acquire", wait=False)
            pvm.wait('running', value="Acquire")
    else:
        print("Nothing to do...")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
