import lib
import math


def isclose(a:float, b:float, precision = 0.0001):
    return (abs(float(a)-float(b)) < precision)


def main():
    factory = lib.RobotFactory()
    robot = factory.create_from_file('JSON', 'dimentions.json')
    
    step = 50

    attempts = 0
    out_of_reach = 0
    error = 0
    success = 0
    conflict = 0

    delta_x = 0.0
    delta_y = 0.0
    delta_z = 0.0
    
    with open('pointcloud.csv', 'w') as file:
        file.write("x, y, z\n")
        for x in range(-1200, 1200, step):
            for y in range(-1200, 1200, step):
                for z in range(-1600, 0, step):
                    attempts += 1
                    try:
                        th = robot.inverse_kinematics(x, y, z, 0, 12)
                    except Exception as e:
                        out_of_reach += 1
                        #print("out of reach: ", e)
                        continue

                    try:
                        I = robot.forward_kinematics(*th, 12)
                    except ValueError:
                        error += 1
                        continue

                    I[0] = -I[0]
                    I[1] = -I[1]
                    delta_x += (I[0]-x)**2
                    delta_y += (I[1]-y)**2
                    delta_z += (I[2]-z)**2
                    if isclose(I[0], x) and isclose(I[1], y) and isclose(I[2], z):
                        success += 1
                        file.write(f"{I[0]}, {I[1]}, {I[2]}\n")
                        #print("Valid!")
                    else:                        
                        conflict += 1
                        """
                        print(f"fwd={[x, y, z]},  inv={I[:3]}")
                        if not isclose(I[0], x):
                            print("x failed")
                        if not isclose(I[1], y):
                            print("y failed")
                        if not isclose(I[2], z):
                            print("z failed")
                        """
    
    assert(attempts == out_of_reach+error+success + conflict)
    print(f"\n Attempts: {attempts} \n Out of reach: {out_of_reach} \n Error: {error} \n Success: {success} \n Conflict: {conflict}")
    reached = attempts-out_of_reach-error
    print(f"\n MSE: \n\t x: {delta_x/reached} \n\t y: {delta_y/reached} \n\t z: {delta_z/reached}")






if __name__ == '__main__':
    main()
