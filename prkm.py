import lib
import argparse

parser = argparse.ArgumentParser(prog="Delta Robot Kinematics")
parser.add_argument('filename')
subparsers = parser.add_subparsers(help='Subcommand help') 

inverse_parser = subparsers.add_parser('inverse')
inverse_parser.add_argument('x', type=float)
inverse_parser.add_argument('y', type=float)
inverse_parser.add_argument('z', type=float)

args = parser.parse_args()

factory = lib.RobotFactory()
robot = factory.create_from_file('JSON', args.filename)
print(robot.inverse_kinematics(args.x, args.y, args.z))
