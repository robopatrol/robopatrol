import subprocess, sys
import json
import rospy
import rospkg
import rosnode
import rosgraph

from robopatrol import srv

ROSPKG = rospkg.RosPack()

class MapService:

    processes = {}

    def __init__(self):
        args = rospy.myargv(argv=sys.argv)

        rospy.init_node('map_service')

        rospy.loginfo("map_service: init")

        self.start_service = rospy.Service(
            'robopatrol/map_service/start',
            srv.MapServiceStart,
            self.handle_start_request
        )
        self.stop_service = rospy.Service(
            'robopatrol/map_service/stop',
            srv.MapServiceStop,
            self.handle_stop_request
        )

        if len(args) > 1:
            self.start_map_server(args[1])

        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def handle_start_request(self, request):
        success = self.start_map_server(request.filename)

        return srv.MapServiceStartResponse(success=success)

    def handle_stop_request(self, request):
        success = self.stop_map_server()

        return srv.MapServiceStopResponse(success=success)

    def start_map_server(self, filename):
        self.stop_map_server()

        map_file = '{0}/maps/{1}'.format(ROSPKG.get_path('robopatrol'), filename)

        rospy.loginfo("map_service: start map server")
        self.processes['map_server'] = subprocess.Popen(['rosrun', 'map_server', 'map_server', map_file])

        try:
            rospy.wait_for_service('/static_map', timeout=5)
            success = True
        except rospy.ROSException, e:
            success = False

        return success

    def stop_map_server(self):
        process = self.processes.pop('map_server', None)
        if process is not None:
            rospy.loginfo("map_service: stop map server")
            process.kill()
            self.cleanup('/map_server')

        return True

    def cleanup(self, node_name):
        master = rosgraph.Master(rosnode.ID)
        nodes_to_cleanup = [n for n in rosnode.get_node_names() if n.startswith(node_name)]
        rosnode.cleanup_master_blacklist(master, nodes_to_cleanup)

    def shutdown(self):
        rospy.loginfo("Shutdown robopatrol map service")
        self.stop_map_server()
        self.map_service.shutdown()
