echo "Installing apt packages"
sudo apt install goby3-apps goby3-gui goby3-moos parallel moos-ivp-apps moos-ivp-gui libmoos-ivp opencpn i2c-tools
echo "Installing pip packages"
pip install python-dateutil plotly pyQt5 h5py
echo "updating PATH"
echo "export PATH=$PATH:$(realpath $PWD/../build/bin)" >> ~/.bashrc