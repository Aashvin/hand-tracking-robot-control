import os
import time


def run_setup():
    print("Please don't close the pop-up window. It will close once finished.")

    time.sleep(3)

    # Make the setup shell script executable
    os.system("chmod +x htrc_setup.sh")

    # Execute the steup shell script
    os.system("./htrc_setup.sh")

    print("Hand Tracking Robot Control setup done!")


if __name__ == "__main__":
    run_setup()
