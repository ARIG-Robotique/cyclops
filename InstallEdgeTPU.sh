
# Add edgetpu repo
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
# Trust google
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo tee /etc/apt/trusted.gpg.d/google.asc

sudo apt-get update

sudo apt-get install libedgetpu1-max libedgetpu-dev libusb-1.0-0-dev

# Connect edgetpu with current user
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1a6e",   ATTRS{idProduct}=="089a", OWNER="$USER", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/coral-local.rules

sudo udevadm control --reload-rules