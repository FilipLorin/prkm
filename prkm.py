import lib
import argparse

parser = argparse.ArgumentParser(prog="python prkm.py")
parser.add_argument('-f', '--file', help='name of JSON file with robot dimentions', required=True)
subparsers = parser.add_subparsers(required=True, help='type kinematic solution to find') 

inverse_parser = subparsers.add_parser('inverse', aliases = ['inv'], help='find inverse kinematics solution for a given tooltip position',
                                       description = """Returns the theta_1-3 and phi joint angles. 
                                       \n Theta=0 when horisontal, positive when downwards. 
                                       \n Phi = 0 ...""")
inverse_parser.add_argument('x', type=float, help='global X coordinate of tooltip to solve for')
inverse_parser.add_argument('y', type=float, help='global Y ...')
inverse_parser.add_argument('z', type=float, help='global Z ...')
inverse_parser.add_argument('a', type=float, help='tool orientation (Z rotation)')
inverse_parser.set_defaults(task='inverse')

forward_parser = subparsers.add_parser('forward',aliases = ['fwd'], help='find forward kinematics solution for given joint positions')
forward_parser.add_argument('th1', type=float, help = 'angle in join 1')
forward_parser.add_argument('th2', type=float, help = 'angle in join 2')
forward_parser.add_argument('th3', type=float, help = 'angle in join 3')
forward_parser.add_argument('phi', type=float, help = 'angle in join 4')
forward_parser.set_defaults(task='forward')

args = parser.parse_args()

factory = lib.RobotFactory()
robot = factory.create_from_file('JSON', args.file)

if(args.task=='inverse'):
    print(robot.inverse_kinematics(args.x, args.y, args.z, args.a))
elif(args.task=='forward'):
    print(robot.forward_kinematics(args.th1, args.th2, args.th3, args.phi))
