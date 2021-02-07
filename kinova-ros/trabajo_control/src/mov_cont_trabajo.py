#! /usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import argparse
import time
import numpy as np

def actual_position(msj):
  global posicion_actual
  posicion_actual=np.asarray(msj.position)[0:6];


def argumentParser(argument):
  global bot
  parser = argparse.ArgumentParser(description='Script de movimiento de KinovaRos para el robot de asistencia en laboratorio')
  parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                    help='j2n6s300, el control y posicion esta hecho especificamente para este tipo de robot')
  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  bot = args_.kinova_robotType

def moveJoint (PoseFinal):
  pub = rospy.Publisher('/' + bot + '/effort_joint_trajectory_controller/command', JointTrajectory, queue_size=1)
  sub = rospy.Subscriber('/' + bot + '/joint_states',JointState, actual_position)
  ComandArticulacion = JointTrajectory()
  PuntInterm = JointTrajectoryPoint()
  ComandArticulacion.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
  PuntInterm.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0,6):
    ComandArticulacion.joint_names.append(bot +'_joint_'+str(i+1))
    PuntInterm.positions.append(PoseFinal[i])
    PuntInterm.velocities.append(0)
    PuntInterm.accelerations.append(0)
    PuntInterm.effort.append(0)
  ComandArticulacion.points.append(PuntInterm)
  rate = rospy.Rate(100)
  aux = 0
  while (aux < 50):
    pub.publish(ComandArticulacion)
    aux = aux + 1
    rate.sleep()

def moveFingers (PoseFinal):
  pub = rospy.Publisher('/' + bot + '/effort_finger_trajectory_controller/command', JointTrajectory, queue_size=1)
  sub = rospy.Subscriber('/' + bot + '/joint_states',JointState, actual_position)
  ComandArticulacion = JointTrajectory()
  PuntInterm = JointTrajectoryPoint()
  ComandArticulacion.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
  PuntInterm.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0,3):
    ComandArticulacion.joint_names.append(bot +'_joint_finger_'+str(i+1))
    PuntInterm.positions.append(PoseFinal[i])
    PuntInterm.velocities.append(0)
    PuntInterm.accelerations.append(0)
    PuntInterm.effort.append(0)
  ComandArticulacion.points.append(PuntInterm)
  rate = rospy.Rate(100)
  aux = 0
  while (aux < 500):
    pub.publish(ComandArticulacion)
    aux = aux + 1
    rate.sleep()

def esperarPosicion(posicionFinal,numeroPosicion):
    err=abs(posicionFinal-posicion_actual)
    compensacion=[0,0,0,0,0,0]
    print err
    print np.any(err[0:3] > 0.2)
    while np.any(err[0:3] > 0.2):
        err=abs(posicion_actual-posicionFinal)-compensacion
        for x in range(3):
            if err[x] >= 2*3.14:
                compensacion[x] = compensacion[x] + 2*3.14
            elif err[x]<0:
	        compensacion[x]=0
        print "esperando posicion " + str(numeroPosicion)
        print err[0:3]

if __name__ == '__main__':
  try:
    rospy.init_node('move_robot_using_trajectory_msg')
    argumentParser(None)
    #allow gazebo to launch
    time.sleep(5)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo() 
    posHome=[2.5,4,3,4.7,3.14,0.0]
    posRecogida=[2.5,4.5,3.4,4.7,3.14,0.0]
    posAlmacenamiento=np.array(([-0.64,3.7,2.7,4.7,3.14,0.0],[-0.84,3.7,2.7,4.7,3.14,0.0],[-1.04,3.7,2.7,4.7,3.14,0.0],[-1.24,3.7,2.7,4.7,3.14,0.0],[-1.34,3.7,2.7,4.7,3.14,0.0],[-1.54,3.7,2.7,4.7,3.14,0.0]))
    abrir=[0,0,0]
    cerrar=[1,1,1]
    moveJoint (posHome);
    esperarPosicion(posHome,0)
    moveFingers(abrir)
    for k in range(1,6,1):
      	#home robots
      	posQuerida=posRecogida
      	moveJoint (posQuerida);
      	esperarPosicion(posQuerida,k)
        moveFingers(cerrar)
	print "position " + str(k)

	posQuerida=posAlmacenamiento[k-1]
      	moveJoint (posQuerida);
      	esperarPosicion(posQuerida,k)
        moveFingers(abrir)
	print "position " + str(k)

      	posQuerida=posHome
      	moveJoint (posQuerida);
      	esperarPosicion(posQuerida,k)
	print "position " + str(k)

    else:
      moveJoint ([0.0,2.9,0.0,1.3,4.2,1.4,0.0])

    moveFingers ([1,1,1])
  except rospy.ROSInterruptException:
    print "el programa se ha detenido por razones externas"
