import yaml

robot_name = 'j2n6s300'
test_text = int(input ("Introduzca el controlador deseado\n(0 para Fabrica, 1 para controlador calculado n1 (par computado), 2 para controlador calcualdo n2 (PID)): "))
if test_text==0:
	joint_p = [5000,5000,5000,500,200,500,500]
	joint_i = [0,0,0,0.,0,0,0]
	joint_d = [0,0,0,0,0,0,0]
	print "0 seleccionado"
elif test_text==1:
	joint_p = [2818,2818,2818,2818,2818,2818,2818]
	joint_i = [0,0,0,0,0,0,0]
	joint_d = [101.38,101.38,101.38,101.38,101.38,101.38,101.38]
	print "1 seleccionado"
elif test_text==2:
	joint_p = [146.6,2924.4,720.3,79.6,202.3,384.6,0]
	joint_i = [0.055,0.055,0.055,0.055,0.055,0.055,0.055]
	joint_d = [0.2198,0.2198,0.2198,0.2198,0.2198,0.2198,0.2198]
	print "2 seleccionado"
else:
	test_text = int(input ("No te he entendido, repitelo!(0, 1, 2): "))

finger_p = [10,10,10]
finger_i = [0,0,0]
finger_d = [0,0,0]
dof = int(robot_name[3])
fingers = int(robot_name[5])
robot_joints = []
finger_joints = []


for i in range(1,dof+1):  
   robot_joints.append(robot_name + '_joint_' + str(i))

for i in range(1,fingers+1):  
   finger_joints.append(robot_name + '_joint_finger_' + str(i))

  ############################ Joint state controller
joint_state_controller = {'joint_state_controller':{'type':'joint_state_controller/JointStateController', 'publish_rate' : 500}}
robot_controllers = joint_state_controller

  ########################### trajectory controllers 

  ######## effort
  #joints
joints =[]
gains = {}
constraints = {}
i = 0
for joint in robot_joints:  
    joints.append(joint)
    gains.update({joint:
                           {'p': joint_p[i], 'i': joint_i[i], 'd': joint_d[i], 'i_clamp': 10}
                        })    
    constraints.update({joint:
                         {
                          'trajectory':0.05,
                          'goal': 0.02
                         }
                       })
    i = i + 1
joints = {'joints': joints}
gains = {'gains': gains}
constraints_dic = {  'goal_time': 1.0,
                         'stopped_velocity_tolerance': 0.02}                   			              
constraints_dic.update(constraints)              
constraints_dic = {'constraints': constraints_dic}
robot_trajectory_position_controller = {'type':'effort_controllers/JointTrajectoryController'}
robot_trajectory_position_controller.update(joints)
robot_trajectory_position_controller.update(gains)
robot_trajectory_position_controller.update(constraints_dic)
robot_trajectory_position_controller_dic = {'effort_joint_trajectory_controller': 
                                               robot_trajectory_position_controller}

robot_controllers.update(robot_trajectory_position_controller_dic)

  #fingers
joints =[]
gains = {}
constraints = {}
i=0
for joint in finger_joints:  
    joints.append(joint)
    gains.update({joint:
                           {'p': finger_p[i], 'i': finger_i[i], 'd': finger_d[i], 'i_clamp': 1}
                        })    
    constraints.update({joint:
                         {
                          'trajectory':0.05,
                          'goal': 0.02
                         }
                       })
    i = i + 1
joints = {'joints': joints}
gains = {'gains': gains}
constraints_dic = {  'goal_time': 1.0,
                         'stopped_velocity_tolerance': 0.02}                   			              
constraints_dic.update(constraints)              
constraints_dic = {'constraints': constraints_dic}
robot_trajectory_position_controller = {'type':'effort_controllers/JointTrajectoryController'}
robot_trajectory_position_controller.update(joints)
robot_trajectory_position_controller.update(gains)
robot_trajectory_position_controller.update(constraints_dic)
robot_trajectory_position_controller_dic = {'effort_finger_trajectory_controller': 
                                               robot_trajectory_position_controller}

robot_controllers.update(robot_trajectory_position_controller_dic)

  ########################### Joint by joint controllers

  #####position
  #add joint position controller
i = 0
for joint in robot_joints:    
    robot_joint_position_controller =  { 'joint_' + str(i+1) + '_position_controller':
										                       {
										                       'type':'effort_controllers/JointPositionController',
										                       'joint':joint,
										                       'pid': {'p': joint_p[i], 'i': joint_i[i], 'd': joint_d[i]}
										                       }
										                   }
    robot_controllers.update(robot_joint_position_controller)
    i = i + 1

  #add finger joint position controllers 
i = 0
for joint in finger_joints:  
    finger_joint_position_controller =  { 'finger_' + str(i+1) + '_position_controller':
										                       {
										                       'type':'effort_controllers/JointPositionController',
										                       'joint':joint,
										                       'pid': {'p': finger_p[i], 'i': finger_i[i], 'd': finger_d[i]}
										                       }
										                   }
    i = i + 1
    robot_controllers.update(finger_joint_position_controller)





config = {robot_name: robot_controllers}

with open(robot_name +'_control.yaml', 'w') as outfile:
      yaml.dump(config, outfile, default_flow_style=False)

