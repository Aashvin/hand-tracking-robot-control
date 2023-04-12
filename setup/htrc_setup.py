import os
import time

print("Please don't close the pop-up window. It will close once finished.")

time.sleep(3)

os.system('chmod +x htrc_setup.sh')

os.system('./htrc_setup.sh')

print("Hand Tracking Robot Control setup done!")