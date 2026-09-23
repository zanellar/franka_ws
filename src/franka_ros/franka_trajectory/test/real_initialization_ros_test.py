#!/usr/bin/env python3
"""ROS handoff integration test using fake manager, action and Franka state."""
import unittest
import actionlib
import rospy
import rostest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (ListControllers, ListControllersResponse,
    LoadController, LoadControllerResponse, SwitchController, SwitchControllerResponse)
from franka_msgs.msg import FrankaState
from franka_msgs.srv import StartDirectionalExperiment, StartDirectionalExperimentResponse
from franka_trajectory.srv import InitializeJointPose, SetLinearCommand

CART='cartesian_impedance_directional_kinetic_energy_cbf_controller'
JOINT='position_joint_trajectory_controller'
Q=[0., -.2, 0., -2.2, 0., 3.2, .785398163397]


class HandoffTest(unittest.TestCase):
    def setUp(self):
        self.q=list(Q)
        self.running={CART:'running'}
        self.switches=[]
        self.commands=[]
        self.goals=[]
        self.fail_action=False
        self.state_pub=rospy.Publisher('/franka_state_controller/franka_states', FrankaState, queue_size=1)
        joints=''
        for i in range(1,8):
            joints+=('<link name="l{0}"/><joint name="fr3_joint{0}" type="revolute">'
                     '<parent link="l{1}"/><child link="l{0}"/><axis xyz="0 0 1"/>'
                     '<limit lower="-5" upper="5" velocity="2" effort="87"/>'
                     '<safety_controller soft_lower_limit="-4.9" soft_upper_limit="4.9" '
                     'k_position="20" k_velocity="10"/></joint>').format(i,i-1)
        rospy.set_param('/robot_description','<robot name="mock"><link name="l0"/>'+joints+'</robot>')
        prefix='/'+CART+'/dynamic_reconfigure_compliance_param_node/'
        rospy.set_param(prefix+'Kmax',.02); rospy.set_param(prefix+'alpha',1.)
        self.services=[rospy.Service('/controller_manager/list_controllers',ListControllers,self.list_controllers),
            rospy.Service('/controller_manager/load_controller',LoadController,self.load),
            rospy.Service('/controller_manager/switch_controller',SwitchController,self.switch),
            rospy.Service('/fake_directional/start_experiment',StartDirectionalExperiment,self.command)]
        self.action=actionlib.SimpleActionServer('/'+JOINT+'/follow_joint_trajectory',
            FollowJointTrajectoryAction,execute_cb=self.goal,auto_start=False)
        self.action.start()
        self.timer=rospy.Timer(rospy.Duration(.01),self.publish)
        rospy.wait_for_service('/trajectory_publisher/initialize_joint_pose',10.)
        self.initialize=rospy.ServiceProxy('/trajectory_publisher/initialize_joint_pose',InitializeJointPose)
        self.experiment=rospy.ServiceProxy('/trajectory_publisher/set_experiment_command',SetLinearCommand)

    def tearDown(self):
        self.timer.shutdown()
        for service in self.services: service.shutdown()
        self.state_pub.unregister()

    def publish(self,_):
        state=FrankaState()
        state.header.stamp=rospy.Time.now()
        state.q=self.q; state.dq=[0.]*7; state.robot_mode=FrankaState.ROBOT_MODE_MOVE
        state.O_T_EE=[1,0,0,0, 0,1,0,0, 0,0,1,0, .3+.01*self.q[0],0,.5,1]
        self.state_pub.publish(state)

    def list_controllers(self,_):
        response=ListControllersResponse()
        for name,state in self.running.items():
            ctrl=ControllerState(name=name,state=state)
            ctrl.type='position_controllers/JointTrajectoryController' if name==JOINT else 'mock/Cartesian'
            response.controller.append(ctrl)
        return response

    def load(self,req):
        self.running[req.name]='stopped'
        return LoadControllerResponse(ok=True)

    def switch(self,req):
        self.switches.append(req)
        for name in req.stop_controllers: self.running[name]='stopped'
        for name in req.start_controllers: self.running[name]='running'
        return SwitchControllerResponse(ok=True)

    def command(self,req):
        self.commands.append(req)
        return StartDirectionalExperimentResponse(success=True,message='accepted by fake controller')

    def goal(self,goal):
        self.goals.append(goal)
        if self.fail_action:
            self.action.set_aborted(FollowJointTrajectoryResult(error_code=-4))
        else:
            self.q=list(goal.trajectory.points[-1].positions)
            self.action.set_succeeded(FollowJointTrajectoryResult(error_code=0))

    def test_real_sequence_rejection_failure_and_recovery(self):
        self.assertFalse(self.experiment(.01,0,0,False,.02,1).success)
        invalid=list(Q); invalid[0]=6
        self.assertFalse(self.initialize(invalid,12.).success)
        self.assertEqual(self.switches,[])
        result=self.initialize(Q,12.)
        self.assertTrue(result.success,result.message)
        self.assertEqual([s.strictness for s in self.switches],[2,2])
        self.assertEqual(list(self.switches[0].start_controllers),[JOINT])
        self.assertEqual(list(self.switches[1].start_controllers),[CART])
        self.assertFalse(self.commands[-1].cbf_active)
        points=self.goals[-1].trajectory.points
        self.assertEqual(list(points[-1].velocities),[0.]*7)
        self.assertEqual(list(points[-1].accelerations),[0.]*7)
        self.assertAlmostEqual(points[-1].time_from_start.to_sec(),12.)
        result=self.experiment(.01,.02,0,True,.02,2.)
        self.assertTrue(result.success,result.message)
        self.assertAlmostEqual(result.applied_pose.position.x,.31)
        self.assertAlmostEqual(result.applied_pose.position.y,.02)
        self.fail_action=True
        self.assertFalse(self.initialize(Q,12.).success)
        self.assertEqual(self.running[CART],'stopped')
        self.assertFalse(self.experiment(.01,0,0,False,.02,1).success)
        self.fail_action=False
        result=self.initialize(Q,12.)
        self.assertTrue(result.success,result.message)
        self.assertTrue(self.experiment(0,0,0,False,.02,1).success)


if __name__=='__main__':
    rospy.init_node('real_initialization_test')
    rostest.rosrun('franka_trajectory','real_initialization_test',HandoffTest)
