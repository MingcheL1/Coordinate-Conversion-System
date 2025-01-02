import numpy as np

#Rotation around each axis; counterclockwise is positive, clockwise is negative
#https://mathworld.wolfram.com/RotationMatrix.html
def rotate_axis_x(alpha):
    rot = np.array([[1, 0, 0], [0, np.cos(alpha), np.sin(alpha)], [0, -np.sin(alpha), np.cos(alpha)]])
    return rot

def rotate_axis_y(beta):
    rot = np.array([[np.cos(beta), 0, -np.sin(beta)], [0, 1, 0], [0, np.sin(beta), np.cos(beta)]])
    return rot

def rotate_axis_z(gamma):
    rot = np.array([[np.cos(gamma), np.sin(gamma), 0], [-np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])
    return rot

def pixel_to_camera(u, v, h, roll, pitch):
    u0 = 780/2
    v0 = 780/2
    dx = 0.00112*4 # we scaled by 4
    dy = dx
    focalL = 3.37
    #let x_c = xc/zc, y_c = yc/zc, z_c = zc/zc
    x_c = -(u-u0)*dx/focalL
    y_c = -(v-v0)*dy/focalL
    z_c = 1

    R_x = rotate_axis_x(np.deg2rad(-(90-pitch)))
    R_y = rotate_axis_y(np.deg2rad(-roll))

    R = R_x @ R_y
    camera_coords = np.array([x_c, y_c, z_c])
    rotated_coords = R @ camera_coords

    x_prime_c = rotated_coords[0]
    y_prime_c = rotated_coords[1]
    z_prime_c = rotated_coords[2]

    scale = -h / y_prime_c
    x_prime_c *= scale
    z_prime_c *= scale
    y_prime_c = -h

    original_camera_coords = np.linalg.inv(R) @ np.array([x_prime_c, -h, z_prime_c])

    return original_camera_coords

def pixel_to_world(u, v, h, roll, pitch):
    u0 = 780/2
    v0 = 780/2
    dx = 0.00112*4 # we scaled by 4
    dy = dx
    focalL = 3.37
    #let x_c = xc/zc, y_c = yc/zc, z_c = zc/zc
    x_c = -(u-u0)*dx/focalL
    y_c = -(v-v0)*dy/focalL
    z_c = 1

    R_x = rotate_axis_x(np.deg2rad(-(90-pitch)))
    R_y = rotate_axis_y(np.deg2rad(-roll))

    R = R_x @ R_y
    camera_coords = np.array([x_c, y_c, z_c])
    rotated_coords = R @ camera_coords

    x_prime_c = rotated_coords[0]
    y_prime_c = rotated_coords[1]
    z_prime_c = rotated_coords[2]

    scale = -h / y_prime_c
    xw = x_prime_c * scale
    yw = 0
    zw = z_prime_c * scale
    return np.array([xw,yw,zw])



def pixel_to_uav(u, v, h, pitch, roll, T):
    camera_pos = pixel_to_camera(u, v, h, roll, pitch)
    uav_pos = rotate_axis_z(np.deg2rad(90)) @ camera_pos + T
    return uav_pos

def pixel_to_ned(u, v, h, pitch, roll, yaw, T):
    r_z = rotate_axis_z(np.deg2rad(yaw))
    r_y = rotate_axis_y(np.deg2rad(pitch))
    r_x = rotate_axis_x(np.deg2rad(roll))
    R = r_x @ r_y @ r_z
    # NED to UAV: Rz(yaw)->Ry(pitch)->Rx(roll)
    # UAV to NED: Rx(-roll)->Ry(-pitch)->Rz(-yaw)
    # inv(R) should be = Rz(-yaw) @ Ry(-pitch) @ Rx(-roll)
    uav_pos = pixel_to_uav(u, v, h, pitch, roll, T)
    ned_pos = np.linalg.inv(R) @ uav_pos
    return ned_pos

# Datum Transformations of GPS Positions (page 3,4)
def pixel_to_GPS(u, v, lat, lon, alt, pitch, roll, yaw, T):
    a = 6378137.0 #semi-major axis 
    b = 6356752.314245
    e = (1-(b**2/a**2))**0.5
    e2 = (a**2/b**2-1)**0.5
    lat = np.deg2rad(lat)
    lon = np.deg2rad(lon)
    hight = alt
    N = a/np.sqrt(1-e*e*np.sin(lat)*np.sin(lat))
    r = np.array([[-np.sin(lat)*np.cos(lon), -np.sin(lon), -np.cos(lat)*np.cos(lon)], 
                  [-np.sin(lat)*np.sin(lon), np.cos(lon), -np.cos(lat)*np.sin(lon)], 
                  [np.cos(lat), 0, -np.sin(lat)]])
    t = [(N+hight)*np.cos(lat)*np.cos(lon), (N+hight)*np.cos(lat)*np.sin(lon), (N*(1-e*e)+hight)*np.sin(lat)]
    ned_pos = pixel_to_ned(u, v, hight, pitch, roll, yaw, T)
    geo_pos = r @ ned_pos + t
    x = geo_pos[0]
    y = geo_pos[1]
    z = geo_pos[2]
    theta = np.arctan(z*a/(b*np.sqrt(x*x+y*y)))
    lon1 = np.arctan(y/x)
    if x < 0 and y > 0:
        lon1 = lon1 + 3.1415926
    elif x < 0 and y < 0:
        lon1 = lon1 - 3.1415926
    lat1 = np.arctan((z+b*e2*e2*np.sin(theta)**3)/(np.sqrt(x*x+y*y)-a*e*e*np.cos(theta)**3))
    hight1 = np.sqrt(x*x+y*y) / np.cos(lat1) - N
    return np.array([np.rad2deg(lat1), np.rad2deg(lon1), hight1])

homeLat = 43.980513
homeLon = -88.574271
hight = 10 # meter
pitch = -5
roll = 2
yaw = 10
u = 15
v = 10
t = [0.1,0,0]

print(pixel_to_world(u,v,hight,roll,pitch))
newLat,newLon,newH = pixel_to_GPS(u,v,homeLat,homeLon,hight,pitch,roll,yaw,t)
print(homeLat,homeLon,hight)
print(newLat,newLon,newH)
