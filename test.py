import lib
import math


def isclose(a:float, b:float, precision):
    return (abs(float(a)-float(b)) < precision)


def main():
    factory = lib.RobotFactory()
    robot = factory.create_from_file('JSON', 'dimentions.json')
    
    step = 50
    precision = 0.001

    attempts = 0
    out_of_reach = 0
    error = 0
    success = 0
    conflict = 0

    delta_x1 = 0.0
    delta_y1 = 0.0
    delta_z1 = 0.0
    delta_x2 = 0.0
    delta_y2 = 0.0
    delta_z2 = 0.0
    

    with open('pointcloud.csv', 'w') as file:
        file.write("x, y, z, th1, th2, th3\n")
        for x in range(-1500, 1500, step):
            for y in range(-1500, 1500, step):
                for z in range(-1500, 0, step):
                    attempts += 1
                    try:
                        th = robot.inverse_kinematics(x, y, z, 0, 8)
                    except Exception as e:
                        out_of_reach += 1
                        #print("out of reach: ", e)
                        continue

                    try:
                        I = robot.forward_kinematics(*th, 7)
                    except ValueError as e:
                        #print("FWD Error:", e)
                        error += 1
                        continue
                    
                    if math.isnan(I[0]) or math.isnan(I[1]) or math.isnan(I[2]):
                        error += 1
                        continue
                    

                    I[0] = -I[0]
                    I[1] = -I[1]
                    delta_x1 += (I[0]-x)**2
                    delta_y1 += (I[1]-y)**2
                    delta_z1 += (I[2]-z)**2

                    if isclose(I[0], x, precision) and isclose(I[1], y, precision) and isclose(I[2], z, precision):
                        success += 1
                        file.write(f"{I[0]}, {I[1]}, {I[2]}, {th[0]}, {th[1]}, {th[2]}\n")
                        delta_x2 += (I[0]-x)**2
                        delta_y2 += (I[1]-y)**2
                        delta_z2 += (I[2]-z)**2
                    else:                        
                        conflict += 1
    
    assert(attempts == out_of_reach+error+success + conflict)
    print(f"\n Attempts: {attempts} \n Out of reach: {out_of_reach} \n Error: {error} \n Success: {success} \n Conflict (not within +/-{precision}): {conflict}")
    reached = attempts - out_of_reach - error
    print(f"\n MSE (all): \n\t x: {delta_x1/reached} \n\t y: {delta_y1/reached} \n\t z: {delta_z1/reached}")
    print(f"\n MSE (within +/-{precision}): \n\t x: {delta_x2/success} \n\t y: {delta_y2/success} \n\t z: {delta_z2/success}")


if __name__ == '__main__':
    main()
