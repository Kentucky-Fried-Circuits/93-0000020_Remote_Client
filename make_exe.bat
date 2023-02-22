pyinstaller -F 93-0000020.py --windowed --hidden-import C:\Users\balano\Documents\GitHub\python-can-j1939\venv2\Lib\site-packages\can\interfaces\pcan\basic.py

pyinstaller -F --add-data "C:\Users\YOUR_USER\Documents\GitHub\python-can-j1939;j1939" 93-0000020.py --windowed