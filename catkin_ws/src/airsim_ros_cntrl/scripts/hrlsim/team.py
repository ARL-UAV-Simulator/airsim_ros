#! /usr/bin/python3

import numpy as np
import rospy, actionlib, math

from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Point, Vector3, TwistStamped
from airsim_ros_cntrl.msg import Multirotor
from airsim_ros_pkgs.srv import Takeoff, Land
from actionlib_msgs.msg import GoalStatus

from std_srvs.srv import SetBool

from airsim_ros_cntrl.msg import (
    TrackObjectAction,
    TrackObjectGoal,
)
from airsim_ros_cntrl.msg import (
    MoveToLocationAction,
    MoveToLocationGoal,
)

from typing import List, Dict

import hrlsim




def setUpTargets(client: hrlsim.airsim.MultirotorClient, target_list: List):
    for t in target_list:
        pose = hrlsim.airsim.Pose(t[2], t[3])
        client.simSetObjectPose(object_name=t[0], pose=pose)


def createDroneTeamLists(client, ip, setupTargets=True):
    if ip != "":
        vehicle_list = ["Drone0"]
        # vehicle_list = ["Drone0", "Drone1", "Drone2", "Target0", "Drone3", "Drone4", "Target1"]
    else:
        vehicle_list = hrlsim.utility.getDroneListFromSettings()

    #vehicle_list = ["Drone0", "Drone1", "Drone2", "Drone3", "Drone4", "Drone5", "Drone6", "Drone7", "Drone8", "Drone9"]
    #vehicle_list = ["Drone0", "Drone1", "Drone2", "Drone3"]

    drone_list = []
    team_list = []
    target_list = []
    target_procs = []
    

    for v in vehicle_list:
        if "Drone" in v:
            drone_list.append(v)

    if setupTargets:
        target_procs = dict()
        target_list = [
            (
                "African_Poacher_1_WalkwRifleLow_Anim2_2",
                [(0,0,19), (0,0,7), (1.5, 0, 3.5), (1.5, -math.pi/10, 3), (1.5, -0.01, 10), (1.5,math.pi/5,1), (1.5, 0, 15), (1.5,-math.pi/5,1), (1.5,-0.02,30)],
                hrlsim.airsim.Vector3r(-250, -312, 0),
                hrlsim.airsim.to_quaternion(0, 0, 2.32),
            ),
        ]
        if(len(drone_list) > 1):    
            target_list.append([
                "African_Poacher_1_WalkwRifleLow_Anim3_11",
                [(0,0,19), (1.5, 0, 8), (1.5, -math.pi/10, 3.5), (1.5, -0.01, 10), (1.5,-math.pi/10,1), (1.5,0,15), (1.5,math.pi/10,1), (1.5,-0.02,30)],
                hrlsim.airsim.Vector3r(-259, -318, 0),
                hrlsim.airsim.to_quaternion(0, 0, 2.32),
            ]
        )
        setUpTargets(client, target_list)

        for i in range(len(target_list)):
            target_procs[target_list[i][0]] = hrlsim.Target(
                "Team" + str(i), target_list[i][0], ip=ip, path=target_list[i][1]
            )
            target_procs[target_list[i][0]].start()

            sub_drone = drone_list[
                i
                * len(drone_list)
                // len(target_list) : (i + 1)
                * len(drone_list)
                // len(target_list)
            ]
            s = Team("Team" + str(i), sub_drone, target_procs[target_list[i][0]])
            team_list.append(s)
    
    else:
        team_list.append(Team("Team0", drone_list, None))

    return team_list, drone_list, target_list, target_procs


class Team:
    """
    Class to define a team of drones.
    A team is comprised of any number of agents and up to one target to track.
    Manages the deployment and management of all drones.
    """

    def __init__(
        self,
        trajType: hrlsim.traj.Trajectory,
        teamName: str,
        vehicle_list: List[str],
        target: hrlsim.Target
    ) -> None:
        """
        Contructs a team object.
        A team is comprised of any number of agents and up to one target to track. 

        Args:
            teamName (str): The name of the team
            vehicle_list (List[str]): List of agent names
            target (Target): The target to track
        """
        self.team_name = teamName
        self.vehicle_list = vehicle_list
        self.centroid_timer = None
        self.goal = None
        self.traj = trajType()

        self.drones: Dict[str, hrlsim.drone.Agent] = dict()

        if target != None:
            self.target = hrlsim.drone.DroneInfo(target.drone_name, target)
        else:
            self.target = None

        self.__shutdown = False

        for i in self.vehicle_list:
            proc = hrlsim.drone.Agent(self.team_name, i, controllerType=hrlsim.controller.LQR, trajType=hrlsim.traj.MinimumSnap)

            self.drones[i] = hrlsim.drone.DroneInfo(i, proc)
            self.drones[i].process.start()

        print("SWARM CREATED WITH %d DRONES" % len(self.drones))

    def setup_ros(self) -> None:
        """
        Function to set up ros topics to talk to teams.
        The team itself does not have a node associated with it.
        """

        self.centroid_des_pub = rospy.Publisher(
            "/"+self.team_name+"/centroid_des/multirotor", Multirotor, queue_size=2
        )
        self.centroid_act_pub = rospy.Publisher(
            "/"+self.team_name+"/centroid_act/multirotor", Multirotor, queue_size=2
        )
        if self.target != None:
            target_prefix = "/" + self.team_name + "/" + self.target.name

            srvs = dict()
            srvs["shutdown"] = rospy.ServiceProxy(
                target_prefix + "/shutdown", SetBool
            )
            self.target.services = srvs

            subs = dict()
            subs["multirotor"] = rospy.Subscriber(target_prefix+"/multirotor", Multirotor, self.target_cb)
            self.target.subs = subs

        for i in self.vehicle_list:

            prefix = self.team_name + "/" + i

            pubs = dict()
            pubs["angle_throttle_cmd"] = rospy.Publisher(prefix + "/angle_throttle_cmd", TwistStamped, queue_size=1)

            takeoff_srv_name = prefix + "/takeoff"
            land_srv_name = prefix + "/land"
            shutdown_srv_name = prefix + "/shutdown"

            rospy.wait_for_service(takeoff_srv_name)
            rospy.wait_for_service(land_srv_name)
            rospy.wait_for_service(shutdown_srv_name)

            srvs = dict()
            srvs["takeoff"] = rospy.ServiceProxy(takeoff_srv_name, Takeoff)
            srvs["land"] = rospy.ServiceProxy(land_srv_name, Land)
            srvs["shutdown"] = rospy.ServiceProxy(shutdown_srv_name, SetBool)

            track_object_action_name = prefix + "/track_object"
            move_to_location_action_name = prefix + "/move_to_location"

            actions = dict()
            actions["track"] = actionlib.SimpleActionClient(
                track_object_action_name, TrackObjectAction
            )
            actions["track"].wait_for_server()

            actions["move_to_location"] = actionlib.SimpleActionClient(
                move_to_location_action_name, MoveToLocationAction
            )
            actions["move_to_location"].wait_for_server()

            subs = dict()
            subs["multirotor"] = rospy.Subscriber(prefix+"/multirotor", Multirotor, self.agent_cb, i)

            self.drones[i].pubs = pubs
            self.drones[i].services = srvs
            self.drones[i].actions = actions
            self.drones[i].subs = subs

    def agent_cb(self, msg: Multirotor, drone_name: str):
        self.drones[drone_name].state = msg

    def target_cb(self, msg: Multirotor):
        self.target.state = msg

                
    def calculateCentroid(self) -> Multirotor:
        centroidPos = np.zeros(3)
        centroidVel = np.zeros(3)
        for drone in self.drones.values():
            pos = drone.state.state.pose.position
            vel = drone.state.state.vel.linear
            centroidPos += np.array([pos.x, pos.y, pos.z])
            centroidVel += np.array([vel.x, vel.y, vel.z])
            

        centroidPos /= len(self.drones.values()) 
        centroidVel /= len(self.drones.values())

        centroid = Multirotor()
        centroid.state.pose.position = Point(*centroidPos)
        centroid.state.vel.linear = Vector3(*centroidVel)
        return centroid

    def activateAgents(self, altitude=-5):
        c = self.calculateCentroid().state.pose.position
        curr_centroid = np.array([[c.x, c.y, c.z]]).T
        ic = np.zeros((3,3))
        fc = np.zeros((3,3))

        goal = curr_centroid + np.array([[0,0,altitude]]).T
        waypoints = np.concatenate((curr_centroid, goal), 1)
        avg_spd = 1.0

        self.traj.generate(waypoints, ic, fc, avg_spd)
        self.t0 = rospy.get_time()

        self.centroid_timer = rospy.Timer(rospy.Duration(0.01), self.centroid_timer_cb, oneshot=False)
        self.track_object(timeout=-1, z_offset=0, object_name="centroid_des")

    def moveInFormation(self, goal, avg_spd):
        # Use ZOBCD gains to move in formation

        '''
        K = np.array([
            [    1.09987239143962,0.0503632378597413,-0.072521395814259,1.53846946911241,0.0631792698420289,-0.0056814103052819,0,0,0,0,0,0 ],
            [    0.0232247807032453,1.12970502886928,0.0197401928657894,-0.0229164941017514,1.72105000761706,-0.0965854744841446,0,0,0,0,0,0 ],
            [    0.0555488153558315,-0.0322472382433944,1.02900257779909,-0.0357679529233441,0.0121304451394579,1.64233208199246,0,0,0,0,0,0 ],
            [    0.0191809004993222,0.119067010657769,-0.0265472209604166,0.0292048266669687,0.0149767888839951,-0.0220930455694303,0.970523660716405,-0.0350395029136082,-0.00587038850809423,1.56151198209515,-0.0456043942696409,0.0853222404629325 ],
            [    -0.0374118207427937,-0.185550358451549,0.146794407191816,0.0416806383530611,0.0205666205159368,-0.113471591824694,-0.0372622583175744,1.15826248753046,0.0929171107613427,0.00457300275993948,1.49506778986822,-0.144762587191591 ],
            [    0.075413204786828,-0.0525702554452755,0.00873072970265868,-0.112686637234897,-0.107550490492135,-0.078642862283437,-0.0825293299541275,0.0355045333520528,1.12236143201042,-0.0931973330849161,0.129508001550065,1.4670967010358 ]
        ])
        '''
        K = np.array([    
            [     0.99287308369909,-0.0152949583045611,0.0604339522725555,1.57628472292529,0.0157136050384367,0.0571201577724558,0,0,0,0,0,0 ],
            [     0.0523610003267187,1.02815900564303,-0.0552806250241997,-0.0941071044463573,1.52038880688991,0.0102372976980017,0,0,0,0,0,0 ],
            [    -0.00672188244961743,-0.0114485209657347,1.03016637499202,0.000484160372958258,-0.00266536361037342,1.51322408595347,0,0,0,0,0,0 ],
            [    -0.0332364406738972,-0.142684026771323,0.0201944559355046,-0.0661919660458793,-0.0526976610133553,-0.0184901447321295,0.99004005480055,0.036854157222961,-0.0432712538315,1.43844792231802,0.0952172422924493,-0.00477767877551061 ],
            [     0.0150948707079744,0.0376857011925861,-0.003816251813846,-0.0334867516662861,0.074344689466647,-0.00608482717914755,0.0479818004916678,1.07213599430373,0.00995046269672629,0.00281588273267293,1.57371988411502,0.0547259546733784 ],
            [    -0.0012734179109349,-0.0690489021053368,0.0218192559866688,0.0245268315546866,-0.00218695486662323,0.00529990582461142,-0.0364455663284159,0.0168213802630929,1.04527692153294,-0.0057403887972013,-0.0230718750280571,1.56459289447182 ]
        ])

        A = np.array([
                [1,	0,	0,	0.1,0,	0,	0,	0,	0,	0,	0,	0,]
                [0,	1,	0,	0,	0.100000000000000,	0,	0,	0,	0,	0,	0,	0,]
                [0,	0,	1,	0,	0,	0.100000000000000,	0,	0,	0,	0,	0,	0,]
                [0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,]
                [0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,]
                [0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,]
                [0,	0,	0,	0,	0,	0,	1,	0,	0,	0.100000000000000,	0,	0,]
                [0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0.100000000000000,	0,]
                [0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0.100000000000000,]
                [0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,]
                [0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,]
                [0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,]
        ])

        B = np.array([
            [0.00500000000000000,	0,	0,	0,	0,	0,]
            [0,	0.00500000000000000,	0,	0,	0,	0,]
            [0,	0,	0.00500000000000000,	0,	0,	0]
            [0.100000000000000,	0,	0,	0,	0,	0,]
            [0,	0.100000000000000,	0,	0,	0,	0,]
            [0,	0,	0.100000000000000,	0,	0,	0]
            [0,	0,	0,	0.00500000000000000,	0,	0,]
            [0,	0,	0,	0,	0.00500000000000000,	0,]
            [0,	0,	0,	0,	0,	0.00500000000000000,]
            [0,	0,	0,	0.100000000000000,	0,	0,]
            [0,	0,	0,	0,	0.100000000000000,	0,]
            [0,	0,	0,	0,	0,	0.100000000000000,]
        ])
        '''
        
        K = np.array([
                [  1.00094740207363,-0.00130062620038081,-0.000340525197942014,1.50989626938464,0.00134116408096208,0.0131583511649842,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [  0.00607075793316134,1.00420468679976,0.0163644061375934,-0.00327279413451607,1.49744814808315,0.00937246010035713,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [ -0.0109461299190366,-0.002414354989365,0.999835489458373,-0.00047879913210995,0.0167415342858258,1.51073018195499,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [ -0.0198057886406471,0.0298992710009796,0.00356429414854945,0.00531824549134384,-0.011803036062807,-0.0190315194978225,0.996153054242466,-0.00521429182504937,-0.0135065276217756,1.51588762999361,0.0115311506823012,-0.0035566024405274,-0.00126323945031804,-0.0182901216434001,-0.00486478108433188,0.00510856928578692,0.00464034180827668,0.0003417739037884,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [  0.00360838952001613,0.00486458909365984,0.0213556244593346,0.00375971237463729,-0.0081986897658822,0.00445536075354084,0.00413083617254301,0.987101696038496,0.0082469466037779,-0.00734045812481669,1.49370840718382,-0.00408772900984182,0.00320429720941742,0.0120984127733506,0.0150083477991048,-0.00774372287827863,-0.0236114160143168,-0.00129620334299642,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [ -0.011005453676738,0.00195374102815147,0.0100279915120047,-0.00295036366356937,-0.0012232369037075,-0.0062873174021069,-0.00393840521984097,-0.00983054444785761,0.992578400804054,0.00541936821108156,-0.00285861340762543,1.49035028961927,0.0157302933106837,-0.00353594099992873,0.00292395784409078,6.5471090271967e-06,0.0082267823041851,-0.00623110107144083,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,1.01620599449453,-0.0100152548911734,0.015593583525042,1.4923274108069,-0.00758320292647031,-0.00834184975803722,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,-0.0118643494399271,1.02138174757344,0.0217132098814323,-0.0128106646706255,1.49549037116648,-0.007811928622467,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,0.00786680613719104,0.00460556787843929,1.02073140763144,-0.00302160322564369,-0.00689873427480516,1.50414971634972,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,-0.0123875980605586,0.00126041184550876,0.00410155261940695,-0.00221885976977343,-0.00413049536152166,0.00712599618037066,1.0207175939081,0.00546036487877271,0.00172718438636395,1.51229661466569,0.00902772395095276,0.00589969561250536,0.0073180449192875,0.00459095722477555,-0.00656712429363248,0.00180184590591789,-0.0245967233728617,0.00636413509376939,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,-0.00611762272001503,0.0112849918547551,0.00891500869957105,-0.012531849754656,-0.00527968421294707,0.00907517864827364,0.00649907206620002,0.989400441474781,0.0260778446657397,0.0206334376962379,1.49744369991627,0.0182278138329468,0.0116371695042917,0.000670091330738842,-0.00397792250219154,-0.00931994162284029,-0.00803528838286748,0.00201161375626325,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,0.0143460066018928,0.017039069161325,0.0057914698027441,-0.0010000739576811,0.00050366715994867,0.00722983407660726,0.0273414427258858,-0.0127943680185086,0.98758152311057,0.0144570317331348,0.00128521338023702,1.49841899320452,0.00226386698370158,-0.0158698623558731,-0.00332203321000634,-0.0131737949280398,0.000981771794192222,-2.85972221599854e-05,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.988256612797595,-0.0101903979958195,0.00970940373058818,1.50725111171774,-0.0292683613760029,0.000920308834558043,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00165220964226607,0.993420003002723,0.0162247066820556,0.00887111637267161,1.51926337239728,0.0144567956324884,0,0,0,0,0,0 ],
                [  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00510547200429263,-0.00477464620236454,1.00482608927942,-0.00777424938150554,0.00769883025628776,1.50517277433033,0,0,0,0,0,0 ],
                [  0.0117555617776694,-0.015063990740633,0.000715233600558763,0.0071607673727945,-0.0283398484624039,0.00955038168937658,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.00149847984381164,0.00549231605046939,0.0168240559065341,-0.0074639348963835,0.024141541119826,0.00116961814824416,0.991048128818499,0.0204804694568658,0.0183603962365595,1.50241902279174,-0.00285998855859033,-0.00249928312314438 ],
                [  0.00531450291377396,-0.0115705929973067,-0.0127293684055912,-0.00216117351902254,-0.00699242441021375,-0.0239613306157704,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0218266568507556,0.00441018121603463,0.0257135390840414,0.0095499372120477,0.0144015304105934,0.0110035339428392,-0.00287501051577479,0.984704362869294,0.00484572325643339,-0.0274421411464387,1.51189066355884,0.000760518501853114 ],
                [ -0.00731490302241569,-0.0303517478261307,0.00954562977638494,0.0115226043159583,0.00767269410514088,0.007152882049212,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00276217743871418,-0.0209160939604988,-0.0128070667552836,0.0328316431590413,-0.0145476902931092,-0.000988944586958113,-0.00355760404621555,0.00757576442135438,1.01274208153224,-0.00331870119611789,0.0111293626438045,1.5065117815842 ]
        ])
        '''
        
        c = self.calculateCentroid().state.pose.position
        curr_centroid = np.array([[c.x, c.y, c.z]]).T
        #ic = np.zeros((3,3))
        #fc = np.zeros((3,3))

        goal = np.array([goal]).T
        #waypoints = np.concatenate((curr_centroid, goal), 1)
        #avg_spd = avg_spd

        #self.traj.generate(waypoints, ic, fc, avg_spd)
        self.t0 = rospy.get_time()
        

        l = 4 * math.pi / 3

        delta_theta = 2 * math.pi / len(self.vehicle_list)

        if len(self.vehicle_list) > 1:
            #r = l / delta_theta
            r = 3
        else:
            r = 0



        sleeper = rospy.Rate(10)
        dt = 1/10

        while(np.linalg.norm(curr_centroid-goal) >= 0.1):
            c = self.calculateCentroid()
            curr_centroid = np.array([[c.state.pose.position.x, c.state.pose.position.y, c.state.pose.position.z]]).T
            curr_vel = np.array([[c.state.vel.linear.x, c.state.vel.linear.y, c.state.vel.linear.z]]).T

            team_state = np.append(curr_centroid, np.zeros((4,1)), 0)
            team_state = np.append(team_state, curr_vel, 0)

            target = np.array([[0,0,-10,1,0,0,0,0,0,0]]).T
            #(target, _) = self.traj.compute(rospy.get_time()-self.t0, team_state)

            target = target[[0,1,2,7,8,9]]

            x0 = np.empty((0,1))
            y = np.empty((0,1))

                            
            for i in range(0, len(self.vehicle_list)):
                desired_state = target + np.array([[r*math.cos(delta_theta*i),r*math.sin(delta_theta*i),0,0,0,0]]).T
                x0 = np.append(x0, desired_state,0)

                pos = self.drones[self.vehicle_list[i]].state.state.pose.position
                vel = self.drones[self.vehicle_list[i]].state.state.vel.linear
                cs = np.array([[pos.x, pos.y, pos.z, vel.x, vel.y, vel.z]]).T
                y = np.append(y, cs, 0)

            mass = 1
            m = 3
            n = 6

            N = len(self.drones)
            x = y-x0

            x0N = np.zeros((x0.shape))
            for i in range(0,N):
                x[[n*i+0, n*i+1, n*i+3, n*i+4]] = x[[n*i+1, n*i+0, n*i+4, n*i+3]]
                x[[n*i+2, n*i+5]] = -x[[n*i+2, n*i+5]]

                x0N[[n*i+0, n*i+1, n*i+2]] = x0N[[n*i+0, n*i+1, n*i+2]] + dt*x0N[[n*i+3, n*i+4, n*i+5]]

           


            FiD = mass*(-K@x + np.kron(np.ones((N,1)), np.array([[0,0,9.8]]).T))
            yN = A@x + B@FiD + x0N

            yawD = np.kron(np.ones((N,1)), 0)

            cyawD = np.cos(yawD)
            syawD = np.sin(yawD)
    
            FD = np.reshape(FiD, (3, N), "F")

            TD = np.reshape(np.diagonal(np.sqrt(FD.T@FD)), (N,1)) # N
            throttle = np.minimum(1, np.maximum(0, TD[:]/16.7176))

            pitchD = np.arctan2(FD[0:1,:].T*cyawD[0:N] + FD[1:2,:].T*syawD[0:N], FD[2:3,:].T)    # rad
            rollD = np.arctan2(FD[0:1,:].T*syawD[0:N] - FD[1:2,:].T*cyawD[0:N], TD[:])            # rad

            tmp = pitchD
            pitchD = rollD
            rollD = tmp



            for i in range(0,len(self.vehicle_list)):
                msg = TwistStamped()

                msg.twist.angular.x = rollD[i]
                msg.twist.angular.y = pitchD[i]
                msg.twist.angular.z = yawD[i]
                msg.twist.linear.z = throttle[i]

                #self.drones[self.vehicle_list[i]].pubs["angle_throttle_cmd"].publish(msg)

                goal = MoveToLocationGoal(
                    target=yN[n*i+0:n*i+3],
                    position_frame=MoveToLocationGoal.GLOBAL_FRAME,
                    timeout=3*dt,
                    speed = np.linalg.norm(yN[n*i+3:n*i+6]),
                    tolerance=0.1,
                    yaw_frame=MoveToLocationGoal.LOCAL_FRAME,
                    yaw=0
                )
                self.drones[self.vehicle_list[i]].actions["move_to_location"].send_goal(goal)

            sleeper.sleep()
            




    def trackTargetInFormation(self, time: float, z_offset: float):
        horizon = 0.1

        t0 = rospy.get_time()

        sleeper = rospy.Rate(1/horizon)      
        self.track_object(timeout=-1, z_offset=-10, object_name="centroid_des")
  
        while not rospy.is_shutdown() and rospy.get_time() - t0 < time:

            tp = self.target.state.state.pose.position
            tv = self.target.state.state.vel.linear
            ta = self.target.state.state.acc.linear

            target_pos = np.array([[tp.x, tp.y, tp.z]]).T
            target_vel = np.array([[tv.x, tv.y, tv.z]]).T
            target_acc = np.array([[ta.x, ta.y, ta.z]]).T

            bias = target_vel*horizon + 0.5*target_acc*horizon**2
            fv = target_vel + target_acc*horizon 
            fa = target_acc
            fj = np.zeros((3,1))

            c = self.calculateCentroid().state.pose.position
            cen_pos = np.array([[c.x, c.y, c.z]]).T
            goal = target_pos + bias + np.array([[0,0,z_offset]]).T

            waypoints = np.concatenate((cen_pos, goal), 1)

            target_spd = np.linalg.norm(target_vel)
            d = np.linalg.norm((cen_pos - target_pos))

            if d < 0.125:
                spd_gain = 0
            else:
                spd_gain = 0.25
            
            avg_spd = target_spd + spd_gain*d
            avg_spd = np.minimum(avg_spd, 3.0)
            avg_spd = np.maximum(avg_spd, 0.1)


            cen_vel = self.calculateCentroid().state.vel.linear
            cen_acc = self.calculateCentroid().state.acc.linear

            iv = np.array([[cen_vel.x, cen_vel.y, cen_vel.z]]).T
            ia = np.array([[cen_acc.x, cen_acc.y, cen_acc.z]]).T
            ij = np.zeros((3,1))
            ic = np.concatenate([iv,ia,ij], 1).T

            fc = np.concatenate([fv,fa,fj], 1).T                

            self.traj.generate(waypoints, ic, fc, avg_spd)
            sleeper.sleep()
        

    def deactivateAgents(self):
        if self.centroid_timer != None:
            self.centroid_timer.shutdown()
        
        self.centroid_timer = None


    def centroid_timer_cb(self, event):
        # Handle desired centroid
        (desiredState, accel) = self.traj.compute(rospy.get_time()-self.t0, np.zeros((10,1)), compute_control=False)

        msg = Multirotor()

        msg.state.pose.position = Point(*desiredState[0:3])
        msg.state.vel.linear = Vector3(*desiredState[7:10])
        msg.state.acc.linear = Vector3(*accel)

        self.centroid_des_pub.publish(msg)

        # Handle actual centroid
        msg = self.calculateCentroid()
        if self.goal == None:
            self.centroid_act_pub.publish(msg)
            return

        elif type(self.goal) != Multirotor:
            print("Unrecognized goal type: " + type(self.goal))
            return

    def getDroneList(self) -> Dict[str, hrlsim.drone.Agent]:
        """
        Gets the dictionary of Agents where key is drone name and value is the agent process

        Returns:
            Dict[str, Agent]: Dictionary of Agents
        """
        return self.drones

    def takeoff(self, wait: bool = False) -> None:
        """
        Send the takeoff command to all agents.

        Args:
            wait (bool, optional): Wait for all drones to complete task. Defaults to False.
        """
        for i in self.vehicle_list:
            try:
                self.drones[i].services["takeoff"](False)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        if wait:
            self.wait()

    def land(self, wait: bool = False) -> None:
        """
        Send land command to all agents.

        Args:
            wait (bool, optional): Wait for all drones to complete task. Defaults to False.
        """
        for i in self.vehicle_list:
            try:
                self.drones[i].services["land"](False)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        # Doesn't Work?
        if wait:
            self.wait()

    def wait(self) -> None:
        for drone in self.vehicle_list:
            for action in self.drones[drone].actions.values():
                if action.get_state() != GoalStatus.LOST:
                    action.wait_for_result()


    def move_to_location(
        self, target: List[float], speed: float, timeout: float, tolerance: float, position_frame = MoveToLocationGoal.GLOBAL_FRAME, yaw_frame = MoveToLocationGoal.GLOBAL_FRAME
    ) -> None:
        """
        Move agents to location in a circle configuration.

        Args:
            target (List[float]): the center location to move the agents to
            timeout (float): the timeout to stop moving
            tolerance (float): the tolerance for the final position
            position_frame (int16): frame of reference for posiion command
            yaw_frame (int16): frame of reference for yaw command
        """

        l = 4 * math.pi / 3

        delta_theta = 2 * math.pi / len(self.vehicle_list)

        if len(self.vehicle_list) > 1:
            r = l / delta_theta
        else:
            r = 0

        i = 0

        for drone in self.vehicle_list:
            position = []
            position.append(target[0] - r * math.cos(delta_theta * i))
            position.append(target[1] - r * math.sin(delta_theta * i))
            position.append(target[2])

            yaw = 0.0

            i += 1

            goal = MoveToLocationGoal(
                target=position,
                position_frame=position_frame,
                timeout=timeout,
                speed = speed,
                tolerance=tolerance,
                yaw_frame=yaw_frame,
                yaw=yaw
            )
            self.drones[drone].actions["move_to_location"].send_goal(goal)

        for drone in self.vehicle_list:
            self.drones[drone].actions["move_to_location"].wait_for_result()

    def track_object(self, timeout: float, z_offset: float, object_name = "") -> None:
        """
        Commands the agents to track a target

        Args:
            object_name (str): the drone name to track, must be Target class
            timeout (float): timeout for the command
            z_offset (float): offset for the z axis
        """

        if object_name == "":
            target_name = self.target.name
        else:
            target_name = object_name

        # Track object in a circle configuration

        l = 4 * math.pi / 3

        delta_theta = 2 * math.pi / len(self.vehicle_list)

        if len(self.vehicle_list) > 1:
            r = l / delta_theta
        else:
            r = 0

        i = 0

        for drone in self.vehicle_list:
            dx = -r * math.cos(delta_theta * i)
            dy = -r * math.sin(delta_theta * i)
            dz = z_offset

            i += 1

            offset = (dx, dy, dz)
            goal = TrackObjectGoal(
                object_name=target_name+"/multirotor", timeout=timeout, offset=offset
            )
            self.drones[drone].actions["track"].send_goal(goal)

    def shutdown(self) -> None:
        """
        Shuts down the swarm
        """
        if self.__shutdown == True:
            return

        self.__shutdown = True

        if not rospy.is_shutdown():
            for i in self.vehicle_list:
                try:
                    self.drones[i].services["shutdown"](True)
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

            try:
                if self.target != None:
                    resp = self.target.services["shutdown"](True)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)


if __name__ == "__main__":

    team = Team("team", ["Drone0", "Drone1"], None)

    ######################################
    #
    #     SETUP ROS

    rospy.init_node("swarm")
    rospy.on_shutdown(team.shutdown)

    team.setup_ros()
    rospy.sleep(1)


    #team.move_to_location([-240,-250,-5], speed=2, timeout=10, tolerance=0.1)
    #team.activateAgents()
    #rospy.sleep(5)

    team.moveInFormation([-250,-250,-5], 1.5)

    rospy.sleep(20)


    team.shutdown()