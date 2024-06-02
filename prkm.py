import lib
import argparse

parser = argparse.ArgumentParser(prog="python prkm.py")
parser.add_argument('-f', '--file', help='name of JSON file with robot dimentions', default="dimentions.json")
parser.add_argument('-p', '--precision', help='Number of digits of solution to display', type=int, default=4)
subparsers = parser.add_subparsers(required=True, help='type kinematic solution to find') 

inverse_parser = subparsers.add_parser('inverse', aliases = ['inv'], help='find inverse kinematics solution for a given tooltip position',
                                       description = """Returns the theta_1-3 and phi joint angles. 
                                       \n Theta=0 when horisontal, positive when downwards. 
                                       \n Phi = 0 ...""")
inverse_parser.add_argument('coords', type=float, help='global XYZA coordinates of tooltip to solve for', nargs=4)
inverse_parser.set_defaults(task='inverse')

forward_parser = subparsers.add_parser('forward',aliases = ['fwd'], help='find forward kinematics solution for given joint positions')
forward_parser.add_argument('angles', type=float, help = 'joint angles 123a to find tooltip position for', nargs=4)
forward_parser.set_defaults(task='forward')

args = parser.parse_args()

factory = lib.RobotFactory()
robot = factory.create_from_file('JSON', args.file)

if(args.task=='inverse'):
    print(robot.inverse_kinematics(*args.coords, args.precision))
elif(args.task=='forward'):
    print(robot.forward_kinematics(*args.angles, args.precision))
