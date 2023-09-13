cyclopsdir=`pwd`
echo $cyclopsdir
cat $cyclopsdir/cyclops.service | sed "s|cyclopsdir|${cyclopsdir}|g" | sudo tee /etc/systemd/system/cyclops.service
sudo chmod 640 /etc/systemd/system/cyclops.service
sudo systemctl daemon-reload
sudo systemctl enable cyclops
sudo systemctl start cyclops
sudo systemctl status cyclops.service