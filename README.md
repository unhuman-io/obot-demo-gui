# obot-demo-gui

Possibly just do the install described at https:://github.com/unhuman-io/obot

or

Build the python api as described here: https://github.com/unhuman-io/motor-realtime/blob/develop/doc/build_python_api.md

Python virtual environments are recommened. For example:
```shell
sudo apt install -y virtualenvwrapper
echo ". /usr/share/virtualenvwrapper/virtualenvwrapper.sh" >> ~/.bashrc
. ~/.bashrc
mkvirtualenv env1
add2virtualenv /usr/share/motor-realtime  # or motor_realtime/build/src for a local copy
```

Then in a python environment that has the module `motor`:
```shell
workon env1
pip install -r requirements.txt
python demo.py
```
