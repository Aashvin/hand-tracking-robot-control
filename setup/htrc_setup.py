import os
import time

print("Please don't close the pop-up window.")

time.sleep(3)

os.system('chmod -R +x htrc_setup.sh')

os.system('./htrc_setup.sh')

print("Hand Tracking Robot Control setup done!")