from eVot import eVot
from time import sleep

e = eVot()

e.connect()


e.calibration_values()
#e.wheels(1,1,3000)
sleep(10)


e.close()
