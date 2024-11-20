#from queue import Queue

import os, sys, glob, re, shutil

import modules.mavlink as mavlink


print("FILE save.py = ", __file__)
BASE_DIR = os.path.join(os.path.dirname(__file__), '../data')

# maybe better? - dir = base/session_%05d_f%05d_p
# now: dir = base/session_%08d_p%d   - session_idx + pid
def make_new_dir():
    dir_re = r'session_(\d+)_'
    base_dir_name = BASE_DIR.rstrip('/')

    dirs = glob.glob(os.path.join(base_dir_name, "session_*"))
    existing_idxs = []
    for dn in dirs:
        bn = os.path.basename(dn)
        m = re.match(dir_re, bn)
        if m:
            idx = int(m.group(1))
            existing_idxs += idx,

    print('existing_idxs=', existing_idxs)
    idx = max(existing_idxs, default=0) + 1

    pid = os.getpid()

    final_dir_name = os.path.join(base_dir_name, "session_%08d_p%d" % (idx, pid))

    if os.path.exists(final_dir_name):
        shutil.rmtree(final_dir_name)

    os.makedirs(final_dir_name, exist_ok=True)

    return final_dir_name



def save_data(SaveQueue):
    print('==== in save_data()')
    if not SaveQueue:
        return

    # current directory name
    curr_dn = None

    while True:
        item = SaveQueue.get()
        SaveQueue.task_done()
        if curr_dn is None:
            curr_dn = make_new_dir()
            print('=================MKDIR')

            # asking each item to save himself into a curr_dn
            # item may create subdirectories if needed, e.g. for frames

        if curr_dn is not None and (type(item) == mavlink.MavlinkMessageItem): #hasattr(item, 'save'): \\only saves mavlink messages
            item.save(curr_dn)
