from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math

START_LAT = 22.5180977
START_LON = 113.9007239

vehicle = None

def get_geo_distance(lat1, lon1, lat2, lon2):
    radius = 6378137.0 # m

    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = radius * c

    return d

def get_location_metres(origin_lat, origin_lon, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*origin_lat/180))

    #New position in decimal degrees
    newlat = origin_lat + (dLat * 180/ math.pi)
    newlon = origin_lon + (dLon * 180/ math.pi)

    return [newlat, newlon];
    #  return [int(newlat*1e7), int(newlon*1e7)];

def fly_to_system_position(id, x, y, z):
    lat, lon = get_location_metres(START_LAT, START_LON, x, y)
    target_distance = get_geo_distance(START_LAT, START_LON, lat, lon)
    if target_distance > 30:
        print(f"target position {target_distance} is to far")
        return

    if vehicle:
        msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            id, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000,
            int(lat*1e7), int(lon*1e7), z, # x, y, z positions (not used)
            0, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        vehicle.send_mavlink(msg)
    else:
        print(f"vehicle {id} not connected!")

def land():
    vehicle.mode = VehicleMode("LAND")

def set_vehicles_to_guided_mode():
    vehicle.mode = VehicleMode("GUIDED")

def vehicles_takeoff(height=1.5):
    vehicle.armed = True
    time.sleep(0.5)
    vehicle.simple_takeoff(height)

import socket
import struct
LED_UDP_PORT = 5005
led_list = {
        "1": "192.168.50.101",
        "2": "192.168.50.72"
        } #modify to fit your situation

led_sockets = {}
print("init led devices")
led_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

def change_led(id, r, g, b):
    led_ip = led_list.get(str(id))
    if led_ip:
        led_sock.sendto(struct.pack('BBB', r, g, b), (led_ip, LED_UDP_PORT))
    else:
        print("led device not connected!")

print("connecting to devices..")
vehicle = connect("0.0.0.0:14550", wait_ready=True)

compass_degree = 10 # !UWB system degree to truth north, this is very important before flight!
vehicle.parameters.set("COMPASS_DEC", math.radians(compass_degree))

TASK_LIST = [
        [[[2, 2, 1.5], [255,255,255]], [[3, 3, 2], [255,255,255]]],
        [[[2.5, 2, 1.5], [255,0,0]], [[3.5, 3, 2], [255,0,0]]],
        [[[2, 2, 1.5], [0,255,0]], [[3, 3, 2], [0,255,0]]],
        [[[2.5, 2, 1.5], [0,0,255]], [[3.5, 3, 2], [0,0,255]]],
        [[[2, 2, 1.5], [255,255,255]], [[3, 3, 2], [255,255,255]]],
        ]

set_vehicles_to_guided_mode() #change to GUIDED MODE first

time.sleep(1)
vehicles_takeoff() # takeoff!
time.sleep(3)

for task in TASK_LIST:
    for index, item in enumerate(task):
        drone_id = index + 1
        x, y, z = item[0]
        r, g, gb = item[1]
        print("exec task", drone_id, "xyz:", x, y , z , "rgb:", r, g, b)
        fly_to_system_position(drone_id, 3, 3, 1.5)
        change_led(drone_id, 255,0,0)
    time.sleep(2)

#  fly_to_system_position(1, 3, 3, 1.5)
#  change_led(2, 255,0,0)

land()
