sudo systemctl stop cyclops
sudo systemctl disable cyclops
sudo rm /etc/systemd/system/cyclops.service
sudo systemctl daemon-reload