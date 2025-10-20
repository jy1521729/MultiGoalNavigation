import time
import mc_sdk_py
import time

app=mc_sdk_py.HighLevel()
print("Initializing...")
app.initRobot("192.168.168.99",43988, "192.168.168.168") #local_ip, local_port, dog_ip
print("Initialization completed")
app.passive()
time.sleep(10)
   
