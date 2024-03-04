import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped, Pose , Transform
from moveit_commander import conversions
from moveit_commander import MoveItCommanderException

class PickAndPlace(object):
    """PickAndPlace"""
    def __init__(self):
        # Inicializa o nó
        super(PickAndPlace, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('PickAndPlace')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Cria os objetos necessários para a MoveItInterface
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "robotiq_2f_85"
                rospy.loginfo(gripper_group_name)
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Inicializando o nó no namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_named_position(self, target):
        arm_group = self.arm_group
        
        # Indo para um dos destinos
        rospy.loginfo("Indo para o destino nomeado " + target)
        # Define o destino
        arm_group.set_named_target(target)
        # Planeja a trajetória
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Executa a trajetória e bloqueia enquanto não termina
        return arm_group.execute(trajectory_message, wait=True)

    def reach_joint_angles(self, tolerance):
        arm_group = self.arm_group
        success = True

        # Obtém as posições articulares atuais
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Imprimindo as posições articulares atuais antes do movimento:")
        for p in joint_positions:
            rospy.loginfo(p)

        # Define a tolerância de articulação objetivo
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Define a configuração alvo das articulações
        if self.degrees_of_freedom == 7:
            joint_positions[0] = pi/2
            joint_positions[1] = 0
            joint_positions[2] = pi/4
            joint_positions[3] = -pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
            joint_positions[6] = 0.2
        elif self.degrees_of_freedom == 6:
            joint_positions[0] = 0
            joint_positions[1] = 0
            joint_positions[2] = pi/2
            joint_positions[3] = pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
        arm_group.set_joint_value_target(joint_positions)
        
        # Planeja e executa em um único comando
        success &= arm_group.go(wait=True)

        # Mostra as posições articulares após o movimento
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Imprimindo as posições articulares atuais após o movimento:")
        for p in new_joint_positions:
            rospy.loginfo(p)
        return success

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Obtém a pose atual e exibe-a
        pose = arm_group.get_current_pose()
        rospy.loginfo("A pose cartesiana atual é:")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        
        # Define a tolerância
        arm_group.set_goal_position_tolerance(tolerance)

        # Define a restrição da trajetória, se houver
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Obtém a Posição Cartesiana atual
        arm_group.set_pose_target(pose)

        # Planeja e executa
        rospy.loginfo("Planejando e indo para a Posição Cartesiana")
        return arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        
        # Só precisamos mover essa junta, pois todas as outras são imitação!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False

    def add_object_to_scene(self):
        # Adiciona um cubo à cena
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = 0.6
        object_pose.pose.position.y = 0.0
        object_pose.pose.position.z = 0.6
        self.scene.add_box(name="my_object", pose=object_pose, size=(0.05, 0.05, 0.05))

    def pick(self, object_name, grasp=[], plan_only=False):
        """Pick the named object. A grasp message, or a list of Grasp messages can also be specified as an argument."""
        if type(grasp) is Grasp:
            return self.arm_group.pick(object_name, grasp, plan_only)
        else:
            return self.arm_group.pick(object_name, grasp, plan_only)


    def place(self, object_name, location=None, plan_only=False):
        """Place the named object at a particular location in the environment or somewhere safe in the world if location is not provided."""
        result = False

        if not location:
            result = self.arm_group.place(object_name, plan_only)
        elif type(location) is PoseStamped:
            old = self.arm_group.get_pose_reference_frame()
            self.arm_group.set_pose_reference_frame(location.header.frame_id)
            result = self.arm_group.place(
                object_name, conversions.pose_to_list(location.pose), plan_only
            )
            self.arm_group.set_pose_reference_frame(old)
        elif type(location) is Pose:
            result = self.arm_group.place(
                object_name, conversions.pose_to_list(location), plan_only
            )
        elif type(location) is PlaceLocation:
            result = self.arm_group.place(
                object_name, conversions.msg_to_string(location), plan_only
            )
        elif type(location) is list:
            if location:
                if type(location[0]) is PlaceLocation:
                    result = self.arm_group.place_locations_list(
                        object_name,
                        [conversions.msg_to_string(x) for x in location],
                        plan_only,
                    )
                elif type(location[0]) is PoseStamped:
                    result = self.arm_group.place_poses_list(
                        object_name,
                        [conversions.msg_to_string(x) for x in location],
                        plan_only,
                    )
                else:
                    raise MoveItCommanderException(
                        "Parameter location must be a Pose, PoseStamped, PlaceLocation, list of PoseStamped, or list of PlaceLocation objects."
                    )
        else:
            raise MoveItCommanderException(
                "Parameter location must be a Pose, PoseStamped, PlaceLocation, list of PoseStamped, or list of PlaceLocation objects."
            )

        return result




def main():
    example = PickAndPlace()

    # Para fins de teste
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        rospy.loginfo("Adicionando objeto à cena...")
        example.add_object_to_scene()

    if success:
        rospy.loginfo("Criando um objeto")
        success &= example.reach_gripper_position(0.5)

    # Adicionando teste para pick e place
    if success:
        rospy.loginfo("Executando operação de Pick")
        success &= example.pick("my_object")

    if success:
        rospy.loginfo("Executando operação de Place")
        success &= example.place("my_object")

    # Para fins de teste
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("O exemplo encontrou um erro.")

if __name__ == '__main__':
    main()
